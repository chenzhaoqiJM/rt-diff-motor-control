/*
 * Standalone differential chassis controller for Spacemit K3 Linux side.
 *
 * This program is adapted from the ROS2 jdbot_k3_esos_control node, but keeps
 * only standard Linux/POSIX dependencies. It creates an RPMsg endpoint, sends
 * velocity commands to the ESOS RCPU motor controller, receives wheel-speed
 * feedback, and integrates simple odometry locally.
 *
 * Protocol with RCPU:
 *   send speed: "dir1,speed1;dir2,speed2"  speed unit: r/s
 *   send cfg:   "CFG,ratio,ff,kp,ki,kd,feedback_enable"
 *   recv fb:    "dir1,speed1_mrs;dir2,speed2_mrs" speed unit: mr/s
 */

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEFAULT_RPMSG_CTRL_DEV "/dev/rpmsg_ctrl0"
#define DEFAULT_RPMSG_DATA_DEV "/dev/rpmsg0"
#define DEFAULT_RPMSG_SERVICE_NAME "rpmsg:motor_ctrl"
#define DEFAULT_RPMSG_LOCAL_ADDR 1003U
#define DEFAULT_RPMSG_REMOTE_ADDR 1002U

#define DEFAULT_SEND_HZ 20.0
#define DEFAULT_CMD_TIMEOUT_SEC 0.4
#define DEFAULT_WHEEL_RADIUS_M 0.0335
#define DEFAULT_WHEEL_BASE_M 0.183
#define DEFAULT_MOTOR1_FACTOR 1.0
#define DEFAULT_MOTOR2_FACTOR 1.0
#define DEFAULT_REDUCTION_RATIO 56.0
#define DEFAULT_FF_FACTOR 0.3
#define DEFAULT_PID_KP 0.05
#define DEFAULT_PID_KI 0.2
#define DEFAULT_PID_KD 0.01
#define DEFAULT_FEEDBACK_ENABLE 1

struct rpmsg_endpoint_info {
    char name[32];
    uint32_t src;
    uint32_t dst;
};

#define RPMSG_CREATE_EPT_IOCTL _IOW(0xb5, 0x1, struct rpmsg_endpoint_info)
#define RPMSG_DESTROY_EPT_IOCTL _IO(0xb5, 0x2)

typedef struct {
    const char *ctrl_dev;
    const char *data_dev;
    const char *service_name;
    uint32_t local_addr;
    uint32_t remote_addr;

    double send_hz;
    double cmd_timeout_sec;
    double wheel_radius_m;
    double wheel_base_m;
    double motor1_factor;
    double motor2_factor;

    double reduction_ratio;
    double ff_factor;
    double pid_kp;
    double pid_ki;
    double pid_kd;
    int feedback_enable;
    int cfg_send_on_startup;

    int interactive;
    double init_v;
    double init_w;
} chassis_config_t;

typedef struct {
    int rpmsg_ctrl_fd;
    int rpmsg_fd;
    volatile sig_atomic_t running;
    pthread_t recv_thread;
    pthread_mutex_t lock;

    chassis_config_t cfg;

    double cmd_v;
    double cmd_w;
    struct timespec last_cmd_time;

    double feedback_v_l;
    double feedback_v_r;
    int feedback_dir_l;
    int feedback_dir_r;

    double odom_x;
    double odom_y;
    double odom_yaw;
    struct timespec last_odom_time;
} chassis_controller_t;

static volatile sig_atomic_t g_stop_requested = 0;

static void signal_handler(int sig)
{
    (void)sig;
    g_stop_requested = 1;
}

static double monotonic_elapsed_sec(const struct timespec *start,
                                    const struct timespec *end)
{
    return (double)(end->tv_sec - start->tv_sec) +
           (double)(end->tv_nsec - start->tv_nsec) / 1000000000.0;
}

static void sleep_period(double hz)
{
    struct timespec ts;
    if (hz <= 0.0) {
        hz = DEFAULT_SEND_HZ;
    }
    ts.tv_sec = (time_t)(1.0 / hz);
    ts.tv_nsec = (long)(((1.0 / hz) - (double)ts.tv_sec) * 1000000000.0);
    nanosleep(&ts, NULL);
}

static void print_usage(const char *prog)
{
    printf("Usage: %s [options]\n", prog);
    printf("\nOptions:\n");
    printf("  -i                 Interactive mode. Read commands from stdin.\n");
    printf("  -v <m/s>           Initial linear velocity. Default: 0.0\n");
    printf("  -w <rad/s>         Initial angular velocity. Default: 0.0\n");
    printf("  -r <meter>         Wheel radius. Default: %.4f\n", DEFAULT_WHEEL_RADIUS_M);
    printf("  -b <meter>         Wheel base. Default: %.4f\n", DEFAULT_WHEEL_BASE_M);
    printf("  -s <hz>            Send frequency. Default: %.1f\n", DEFAULT_SEND_HZ);
    printf("  -t <sec>           Command timeout. Default: %.2f\n", DEFAULT_CMD_TIMEOUT_SEC);
    printf("  --no-cfg           Do not send CFG on startup.\n");
    printf("  --no-feedback      Disable RCPU feedback by CFG.\n");
    printf("  -h, --help         Show this help.\n");
    printf("\nInteractive commands:\n");
    printf("  cmd <v_mps> <w_radps>    Set chassis velocity.\n");
    printf("  stop                     Stop chassis.\n");
    printf("  cfg <ratio> <ff> <kp> <ki> <kd> <fb0_or_1>\n");
    printf("  odom                     Print current odometry.\n");
    printf("  quit                     Stop and exit.\n");
}

static void config_init(chassis_config_t *cfg)
{
    cfg->ctrl_dev = DEFAULT_RPMSG_CTRL_DEV;
    cfg->data_dev = DEFAULT_RPMSG_DATA_DEV;
    cfg->service_name = DEFAULT_RPMSG_SERVICE_NAME;
    cfg->local_addr = DEFAULT_RPMSG_LOCAL_ADDR;
    cfg->remote_addr = DEFAULT_RPMSG_REMOTE_ADDR;
    cfg->send_hz = DEFAULT_SEND_HZ;
    cfg->cmd_timeout_sec = DEFAULT_CMD_TIMEOUT_SEC;
    cfg->wheel_radius_m = DEFAULT_WHEEL_RADIUS_M;
    cfg->wheel_base_m = DEFAULT_WHEEL_BASE_M;
    cfg->motor1_factor = DEFAULT_MOTOR1_FACTOR;
    cfg->motor2_factor = DEFAULT_MOTOR2_FACTOR;
    cfg->reduction_ratio = DEFAULT_REDUCTION_RATIO;
    cfg->ff_factor = DEFAULT_FF_FACTOR;
    cfg->pid_kp = DEFAULT_PID_KP;
    cfg->pid_ki = DEFAULT_PID_KI;
    cfg->pid_kd = DEFAULT_PID_KD;
    cfg->feedback_enable = DEFAULT_FEEDBACK_ENABLE;
    cfg->cfg_send_on_startup = 1;
    cfg->interactive = 0;
    cfg->init_v = 0.0;
    cfg->init_w = 0.0;
}

static int parse_args(int argc, char **argv, chassis_config_t *cfg)
{
    int i;
    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0) {
            cfg->interactive = 1;
        } else if (strcmp(argv[i], "-v") == 0 && i + 1 < argc) {
            cfg->init_v = atof(argv[++i]);
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            cfg->init_w = atof(argv[++i]);
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            cfg->wheel_radius_m = atof(argv[++i]);
        } else if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) {
            cfg->wheel_base_m = atof(argv[++i]);
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            cfg->send_hz = atof(argv[++i]);
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            cfg->cmd_timeout_sec = atof(argv[++i]);
        } else if (strcmp(argv[i], "--no-cfg") == 0) {
            cfg->cfg_send_on_startup = 0;
        } else if (strcmp(argv[i], "--no-feedback") == 0) {
            cfg->feedback_enable = 0;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 1;
        } else {
            fprintf(stderr, "Unknown or incomplete option: %s\n", argv[i]);
            print_usage(argv[0]);
            return -1;
        }
    }
    return 0;
}

static int rpmsg_init(chassis_controller_t *ctl)
{
    struct rpmsg_endpoint_info epinfo;

    ctl->rpmsg_ctrl_fd = open(ctl->cfg.ctrl_dev, O_RDWR);
    if (ctl->rpmsg_ctrl_fd < 0) {
        fprintf(stderr, "open %s failed: %s\n", ctl->cfg.ctrl_dev, strerror(errno));
        return -1;
    }

    memset(&epinfo, 0, sizeof(epinfo));
    strncpy(epinfo.name, ctl->cfg.service_name, sizeof(epinfo.name) - 1);
    epinfo.src = ctl->cfg.local_addr;
    epinfo.dst = ctl->cfg.remote_addr;

    if (ioctl(ctl->rpmsg_ctrl_fd, RPMSG_CREATE_EPT_IOCTL, &epinfo) < 0) {
        fprintf(stderr, "create rpmsg endpoint failed: %s\n", strerror(errno));
        close(ctl->rpmsg_ctrl_fd);
        ctl->rpmsg_ctrl_fd = -1;
        return -1;
    }

    ctl->rpmsg_fd = open(ctl->cfg.data_dev, O_RDWR);
    if (ctl->rpmsg_fd < 0) {
        fprintf(stderr, "open %s failed: %s\n", ctl->cfg.data_dev, strerror(errno));
        close(ctl->rpmsg_ctrl_fd);
        ctl->rpmsg_ctrl_fd = -1;
        return -1;
    }

    printf("RPMsg ready: service=%s src=%u dst=%u\n",
           ctl->cfg.service_name, ctl->cfg.local_addr, ctl->cfg.remote_addr);
    return 0;
}

static void rpmsg_cleanup(chassis_controller_t *ctl)
{
    if (ctl->rpmsg_fd >= 0) {
        close(ctl->rpmsg_fd);
        ctl->rpmsg_fd = -1;
    }
    if (ctl->rpmsg_ctrl_fd >= 0) {
        ioctl(ctl->rpmsg_ctrl_fd, RPMSG_DESTROY_EPT_IOCTL);
        close(ctl->rpmsg_ctrl_fd);
        ctl->rpmsg_ctrl_fd = -1;
    }
}

static int send_raw(chassis_controller_t *ctl, const char *msg)
{
    ssize_t ret;
    if (ctl->rpmsg_fd < 0) {
        return -1;
    }

    ret = write(ctl->rpmsg_fd, msg, strlen(msg) + 1);
    if (ret < 0) {
        fprintf(stderr, "rpmsg write failed: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

static int send_cfg(chassis_controller_t *ctl)
{
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "CFG,%.3f,%.3f,%.3f,%.3f,%.3f,%d",
             ctl->cfg.reduction_ratio, ctl->cfg.ff_factor, ctl->cfg.pid_kp,
             ctl->cfg.pid_ki, ctl->cfg.pid_kd, ctl->cfg.feedback_enable ? 1 : 0);

    printf("Send CFG: %s\n", cmd);
    return send_raw(ctl, cmd);
}

static void velocity_to_motor(const chassis_config_t *cfg, double v,
                              int *dir, double *speed_rps)
{
    if (fabs(v) < 1e-3) {
        *dir = 0;
        *speed_rps = 0.0;
        return;
    }

    *dir = v > 0.0 ? 1 : 2;
    *speed_rps = fabs(v) / (2.0 * M_PI * cfg->wheel_radius_m);
}

static int send_chassis_command(chassis_controller_t *ctl, double v, double w)
{
    double v_l = v - w * ctl->cfg.wheel_base_m / 2.0;
    double v_r = v + w * ctl->cfg.wheel_base_m / 2.0;
    int dir1, dir2;
    double speed1, speed2;
    char cmd[96];

    velocity_to_motor(&ctl->cfg, v_l, &dir1, &speed1);
    velocity_to_motor(&ctl->cfg, v_r, &dir2, &speed2);
    speed1 *= ctl->cfg.motor1_factor;
    speed2 *= ctl->cfg.motor2_factor;

    snprintf(cmd, sizeof(cmd), "%d,%.3f;%d,%.3f", dir1, speed1, dir2, speed2);
    return send_raw(ctl, cmd);
}

static void update_odometry(chassis_controller_t *ctl, double v_l, double v_r)
{
    struct timespec now;
    double dt;
    double v;
    double w;

    clock_gettime(CLOCK_MONOTONIC, &now);
    dt = monotonic_elapsed_sec(&ctl->last_odom_time, &now);
    ctl->last_odom_time = now;
    if (dt <= 0.0 || dt > 1.0) {
        return;
    }

    v = (v_l + v_r) / 2.0;
    w = (v_r - v_l) / ctl->cfg.wheel_base_m;

    ctl->odom_yaw += w * dt;
    ctl->odom_x += v * cos(ctl->odom_yaw) * dt;
    ctl->odom_y += v * sin(ctl->odom_yaw) * dt;
}

static void parse_feedback(chassis_controller_t *ctl, const char *buf)
{
    int dir1 = 0, dir2 = 0;
    int speed1_mrs = 0, speed2_mrs = 0;
    double rps1, rps2, v1, v2;

    if (sscanf(buf, "%d,%d;%d,%d", &dir1, &speed1_mrs, &dir2, &speed2_mrs) != 4) {
        printf("[RPMsg] %s\n", buf);
        return;
    }

    rps1 = (double)speed1_mrs / 1000.0;
    rps2 = (double)speed2_mrs / 1000.0;
    v1 = rps1 * 2.0 * M_PI * ctl->cfg.wheel_radius_m;
    v2 = rps2 * 2.0 * M_PI * ctl->cfg.wheel_radius_m;

    if (dir1 == 2) {
        v1 = -v1;
    } else if (dir1 == 0) {
        v1 = 0.0;
    }

    if (dir2 == 2) {
        v2 = -v2;
    } else if (dir2 == 0) {
        v2 = 0.0;
    }

    pthread_mutex_lock(&ctl->lock);
    ctl->feedback_dir_l = dir1;
    ctl->feedback_dir_r = dir2;
    ctl->feedback_v_l = v1;
    ctl->feedback_v_r = v2;
    update_odometry(ctl, v1, v2);
    pthread_mutex_unlock(&ctl->lock);
}

static void *recv_thread_entry(void *arg)
{
    chassis_controller_t *ctl = (chassis_controller_t *)arg;
    char recv_buf[256];
    struct pollfd pfd;
    int print_count = 0;

    memset(&pfd, 0, sizeof(pfd));
    pfd.fd = ctl->rpmsg_fd;
    pfd.events = POLLIN;

    while (ctl->running && !g_stop_requested) {
        int ret = poll(&pfd, 1, 100);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            fprintf(stderr, "poll failed: %s\n", strerror(errno));
            break;
        }
        if (ret == 0 || !(pfd.revents & POLLIN)) {
            continue;
        }

        memset(recv_buf, 0, sizeof(recv_buf));
        ret = (int)read(ctl->rpmsg_fd, recv_buf, sizeof(recv_buf) - 1);
        if (ret < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }
            fprintf(stderr, "read failed: %s\n", strerror(errno));
            break;
        }

        if (ret > 0) {
            parse_feedback(ctl, recv_buf);
            if (++print_count >= 10) {
                pthread_mutex_lock(&ctl->lock);
                printf("[FB] vl=%.3f m/s vr=%.3f m/s | odom x=%.3f y=%.3f yaw=%.3f\n",
                       ctl->feedback_v_l, ctl->feedback_v_r,
                       ctl->odom_x, ctl->odom_y, ctl->odom_yaw);
                pthread_mutex_unlock(&ctl->lock);
                print_count = 0;
            }
        }
    }

    return NULL;
}

static void set_command(chassis_controller_t *ctl, double v, double w)
{
    pthread_mutex_lock(&ctl->lock);
    ctl->cmd_v = v;
    ctl->cmd_w = w;
    clock_gettime(CLOCK_MONOTONIC, &ctl->last_cmd_time);
    pthread_mutex_unlock(&ctl->lock);
}

static void print_odom(chassis_controller_t *ctl)
{
    pthread_mutex_lock(&ctl->lock);
    printf("odom: x=%.4f y=%.4f yaw=%.4f | fb_l=%.4f fb_r=%.4f\n",
           ctl->odom_x, ctl->odom_y, ctl->odom_yaw,
           ctl->feedback_v_l, ctl->feedback_v_r);
    pthread_mutex_unlock(&ctl->lock);
}

static void *stdin_thread_entry(void *arg)
{
    chassis_controller_t *ctl = (chassis_controller_t *)arg;
    char line[256];

    printf("Interactive mode started. Type 'help' for commands.\n");
    while (ctl->running && !g_stop_requested && fgets(line, sizeof(line), stdin) != NULL) {
        char op[32];
        double a, b, c, d, e;
        int fb;

        if (sscanf(line, "%31s", op) != 1) {
            continue;
        }

        if (strcmp(op, "cmd") == 0) {
            if (sscanf(line, "%*s %lf %lf", &a, &b) == 2) {
                set_command(ctl, a, b);
                printf("cmd_vel: v=%.3f m/s w=%.3f rad/s\n", a, b);
            } else {
                printf("Usage: cmd <v_mps> <w_radps>\n");
            }
        } else if (strcmp(op, "stop") == 0) {
            set_command(ctl, 0.0, 0.0);
            send_chassis_command(ctl, 0.0, 0.0);
            printf("stopped\n");
        } else if (strcmp(op, "cfg") == 0) {
            if (sscanf(line, "%*s %lf %lf %lf %lf %lf %d", &a, &b, &c, &d, &e, &fb) == 6) {
                pthread_mutex_lock(&ctl->lock);
                ctl->cfg.reduction_ratio = a;
                ctl->cfg.ff_factor = b;
                ctl->cfg.pid_kp = c;
                ctl->cfg.pid_ki = d;
                ctl->cfg.pid_kd = e;
                ctl->cfg.feedback_enable = fb ? 1 : 0;
                pthread_mutex_unlock(&ctl->lock);
                send_cfg(ctl);
            } else {
                printf("Usage: cfg <ratio> <ff> <kp> <ki> <kd> <fb0_or_1>\n");
            }
        } else if (strcmp(op, "odom") == 0) {
            print_odom(ctl);
        } else if (strcmp(op, "help") == 0) {
            print_usage("k3_chassis_control");
        } else if (strcmp(op, "quit") == 0 || strcmp(op, "exit") == 0) {
            g_stop_requested = 1;
            break;
        } else {
            printf("Unknown command: %s\n", op);
        }
    }

    return NULL;
}

int main(int argc, char **argv)
{
    chassis_controller_t ctl;
    pthread_t stdin_thread;
    int stdin_thread_started = 0;
    int parse_ret;

    memset(&ctl, 0, sizeof(ctl));
    ctl.rpmsg_ctrl_fd = -1;
    ctl.rpmsg_fd = -1;
    ctl.running = 1;
    pthread_mutex_init(&ctl.lock, NULL);
    config_init(&ctl.cfg);

    parse_ret = parse_args(argc, argv, &ctl.cfg);
    if (parse_ret > 0) {
        pthread_mutex_destroy(&ctl.lock);
        return 0;
    }
    if (parse_ret < 0) {
        pthread_mutex_destroy(&ctl.lock);
        return 1;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    clock_gettime(CLOCK_MONOTONIC, &ctl.last_cmd_time);
    clock_gettime(CLOCK_MONOTONIC, &ctl.last_odom_time);
    ctl.cmd_v = ctl.cfg.init_v;
    ctl.cmd_w = ctl.cfg.init_w;

    if (rpmsg_init(&ctl) != 0) {
        pthread_mutex_destroy(&ctl.lock);
        return 1;
    }

    if (pthread_create(&ctl.recv_thread, NULL, recv_thread_entry, &ctl) != 0) {
        fprintf(stderr, "create receive thread failed\n");
        rpmsg_cleanup(&ctl);
        pthread_mutex_destroy(&ctl.lock);
        return 1;
    }

    if (ctl.cfg.cfg_send_on_startup) {
        send_cfg(&ctl);
    }

    if (ctl.cfg.interactive) {
        if (pthread_create(&stdin_thread, NULL, stdin_thread_entry, &ctl) == 0) {
            stdin_thread_started = 1;
        } else {
            fprintf(stderr, "create stdin thread failed, continue without interactive mode\n");
        }
    }

    printf("Controller started. Press Ctrl+C to exit.\n");
    while (!g_stop_requested) {
        struct timespec now;
        double v;
        double w;
        double age;

        pthread_mutex_lock(&ctl.lock);
        clock_gettime(CLOCK_MONOTONIC, &now);
        age = monotonic_elapsed_sec(&ctl.last_cmd_time, &now);
        if (age > ctl.cfg.cmd_timeout_sec) {
            v = 0.0;
            w = 0.0;
        } else {
            v = ctl.cmd_v;
            w = ctl.cmd_w;
        }
        pthread_mutex_unlock(&ctl.lock);

        send_chassis_command(&ctl, v, w);
        sleep_period(ctl.cfg.send_hz);
    }

    ctl.running = 0;
    send_chassis_command(&ctl, 0.0, 0.0);
    pthread_join(ctl.recv_thread, NULL);
    if (stdin_thread_started) {
        pthread_cancel(stdin_thread);
        pthread_join(stdin_thread, NULL);
    }
    rpmsg_cleanup(&ctl);
    pthread_mutex_destroy(&ctl.lock);

    printf("Controller stopped.\n");
    return 0;
}
