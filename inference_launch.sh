#!/bin/bash
# inference_launch.sh — 一键拉起 AgileX 双臂真机的「公共基础设施」终端(对应 docs/cobot_magic_set_up.md §3)。
#
# 默认起这三样(每个一个终端/标签):
#   终端1 roscore  : bash ~/start.sh  (CAN 配置,内部 sudo,密码 agx) -> roscore
#   终端2 camera   : 等 roscore 起来 -> roslaunch astra_camera multi_camera.launch
#   终端3 observe  : 等 roscore 起来 -> rqt_image_view (看相机画面)
# 可选加一项:
#   终端4 arms     : 等 roscore 起来 -> roslaunch piper start_ms_piper.launch mode:=0 auto_enable:=false
#
# 它【不】起 client/server —— 那些要你改完参数后自己起,例如:
#   # FastWAM 推理部署
#   conda activate fastwam
#   PYTHONPATH=src python scripts/deploy_real_bot/serve_fastwam_policy.py  --config-name real_deploy_rtc_fastwam
#   PYTHONPATH=src python scripts/deploy_real_bot/deploy_real_rtc_fastwam.py --config-name real_deploy_rtc_fastwam
#   # 数据采集(还需主从臂: roslaunch piper start_ms_piper.launch mode:=0 auto_enable:=false)
#   conda activate Aloha && python camera_collect_data.py

# 用法:
#   ./inference_launch.sh          # 默认 gnome-terminal:一个窗口三个标签(适合 AnyDesk/本地桌面)
#   ./inference_launch.sh tmux     # 改用 tmux 会话(SSH/无桌面更稳),起好后: tmux attach -t infer
#   ./inference_launch.sh --with-arms
#   ./inference_launch.sh tmux --with-arms
#
# 可选参数:
#   --with-arms   同时拉起双臂后端(roslaunch piper start_ms_piper.launch ...)
#
# 说明:每个终端跑的是交互式 bash(-i 会 source ~/.bashrc,自带完整 ROS 环境:
#       /opt/ros/noetic + camera_ws + piper 工作区都已在 .bashrc 里 source);
#       命令跑完用 exec bash 保持终端不关闭,方便看日志 / Ctrl-C 停。

set -uo pipefail

MODE="gnome"
WITH_ARMS=0

for arg in "$@"; do
  case "$arg" in
    gnome|tmux)
      MODE="$arg"
      ;;
    --with-arms)
      WITH_ARMS=1
      ;;
    *)
      echo "未知参数: $arg"
      echo "用法: ./inference_launch.sh [gnome|tmux] [--with-arms]"
      exit 1
      ;;
  esac
done

# —— 三个终端各自要跑的命令字符串(注意:只用双引号,内部不出现单引号,方便 tmux 再包一层)——
CMD1='echo "[1] start.sh — CAN 配置中,如提示请输 sudo 密码: agx"; bash ~/start.sh; echo "[1] 启动 roscore..."; roscore; exec bash'
WAIT='echo "  等待 roscore 起来..."; until rostopic list >/dev/null 2>&1; do sleep 0.5; done'
CMD2="$WAIT; echo \"[2] roscore 已就绪 — 启动三相机 multi_camera.launch\"; roslaunch astra_camera multi_camera.launch; exec bash"
CMD3="$WAIT; sleep 3; echo \"[3] 启动 rqt_image_view(正常应有 6 个画面可选:3 color + 3 depth)\"; rqt_image_view; exec bash"
CMD4="$WAIT; echo \"[4] roscore 已就绪 — 启动双臂后端 start_ms_piper.launch\"; roslaunch piper start_ms_piper.launch mode:=0 auto_enable:=false; exec bash"

case "$MODE" in
  tmux)
    command -v tmux >/dev/null || { echo "没装 tmux"; exit 1; }
    SESSION=infer
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    tmux new-session -d -s "$SESSION" -n roscore "bash -ic '$CMD1'"
    tmux new-window     -t "$SESSION" -n camera  "bash -ic '$CMD2'"
    tmux new-window     -t "$SESSION" -n observe "bash -ic '$CMD3'"
    if [[ "$WITH_ARMS" -eq 1 ]]; then
      tmux new-window   -t "$SESSION" -n arms    "bash -ic '$CMD4'"
      echo "[launch] tmux 会话 '$SESSION' 已起(4 个 window:roscore/camera/observe/arms)。"
    else
      echo "[launch] tmux 会话 '$SESSION' 已起(3 个 window:roscore/camera/observe)。"
    fi
    echo "[launch]   进去看/操作: tmux attach -t $SESSION"
    if [[ "$WITH_ARMS" -eq 1 ]]; then
      echo "[launch]   切 window: Ctrl-b 然后按 0/1/2/3   脱离(不停): Ctrl-b 然后 d"
    else
      echo "[launch]   切 window: Ctrl-b 然后按 0/1/2   脱离(不停): Ctrl-b 然后 d"
    fi
    echo "[launch]   全部关掉:  tmux kill-session -t $SESSION"
    echo "[launch] 注意:roscore 那个 window 会提示输 sudo 密码 agx,先 attach 进去输。"
    ;;
  gnome|*)
    command -v gnome-terminal >/dev/null || { echo "没装 gnome-terminal,改用: ./inference_launch.sh tmux"; exit 1; }
    # 先开主窗口(终端1),随后两个 --tab 会贴到同一窗口里(GNOME Terminal 3.36 行为)。
    gnome-terminal --window --title="1·roscore" -- bash -ic "$CMD1"
    sleep 1
    gnome-terminal --tab    --title="2·camera"  -- bash -ic "$CMD2"
    sleep 1
    gnome-terminal --tab    --title="3·observe" -- bash -ic "$CMD3"
    if [[ "$WITH_ARMS" -eq 1 ]]; then
      sleep 1
      gnome-terminal --tab  --title="4·arms"    -- bash -ic "$CMD4"
      echo "[launch] 已开 4 个终端(标签 1·roscore / 2·camera / 3·observe / 4·arms)。"
    else
      echo "[launch] 已开 3 个终端(标签 1·roscore / 2·camera / 3·observe)。"
    fi
    echo "[launch] 终端 1 会提示输 sudo 密码: agx ;相机/观测会自动等 roscore 起来。"
    if [[ "$WITH_ARMS" -eq 1 ]]; then
      echo "[launch] 双臂后端已自动拉起(标签 4·arms)。"
    else
      echo "[launch] 如需自动拉起双臂后端,加参数: --with-arms"
    fi
    echo "[launch] client/server 请自己改完参数后启动(见本脚本顶部注释)。"
    ;;
esac
