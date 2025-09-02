import streamlit as st
import rclpy
from ros_manager import RobotDataManager, ControlMode, RosState, RobotData, RosAction
import time

SPACE = '&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;'
SPACE2 = '&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;'

# 初始化机器人数据管理器
robot_data_manager = RobotDataManager(need_create=False)

# 设置页面配置
st.set_page_config(
    page_icon="🔧",
    layout="wide",
    initial_sidebar_state="expanded"
)

# 侧边栏选择功能 - 使用radio按钮替代普通按钮
st.sidebar.title("系统导航")

# 使用radio按钮，它会自动保持状态
# 添加自定义CSS来美化radio按钮
st.markdown("""
<style>
    div.row-widget.stRadio > div {
        flex-direction: column;
        gap: 20px;  /* 增加选项之间的间距 */
    }
    div.row-widget.stRadio > div[role="radiogroup"] > label {
        padding: 15px 10px;  /* 增加按钮内边距使其更大 */
        background-color: #f0f0f0;  /* 浅灰色背景 */
        border-radius: 8px;  /* 圆角 */
        text-align: center;  /* 文字居中 */
        font-size: 16px;  /* 更大的字号 */
        font-weight: bold;  /* 加粗文字 */
        box-shadow: 0 2px 5px rgba(0,0,0,0.1);  /* 轻微阴影效果 */
        transition: all 0.3s;  /* 平滑过渡效果 */
    }
    div.row-widget.stRadio > div[role="radiogroup"] > label:hover {
        background-color: #e0e0e0;  /* 悬停时颜色变化 */
        box-shadow: 0 2px 8px rgba(0,0,0,0.2);  /* 悬停时阴影加深 */
    }
    div.row-widget.stRadio > div[role="radiogroup"] > label[data-baseweb="radio"] > div:first-child {
        background-color: transparent !important;  /* 隐藏原有的radio圆点背景 */
    }
</style>
""", unsafe_allow_html=True)
page = st.sidebar.radio(
    " ",
    ["系 统 控 制", "运 行 监 控"],
    index   = 0,
    key     = "navigation"
)


# 根据radio按钮的选择显示不同页面
if page == "系 统 控 制":
    st.title("Omni Robot 控制面板")

    st.markdown("### 传感器")
    col11, col12 = st.columns([1, 4])
    with col11:
        if st.button("  启动传感器  "):
            robot_data_manager.read_from_memory()
            robot_data_manager.data.ros_action["sensors"] = RosAction.START
            robot_data_manager.update()
    with col12:
        sensors_state_placeholder = st.empty()

    st.markdown("### 底盘")
    col21, col22 = st.columns([1, 4])
    with col21:
        if st.button("局部坐标系模式"):
            robot_data_manager.read_from_memory()
            robot_data_manager.data.ros_action["chassis1"] = RosAction.START
            robot_data_manager.update()
    with col22:
        chassis1_state_placeholder = st.empty()
    col31, col32 = st.columns([1, 4])
    with col31:
        if st.button("全局坐标系模式"):
            robot_data_manager.read_from_memory()
            robot_data_manager.data.ros_action["chassis2"] = RosAction.START
            robot_data_manager.update()
    with col32:
        chassis2_state_placeholder = st.empty()


    st.markdown("### 手柄")
    col41, col42 = st.columns([1, 4])
    with col41:
        if st.button("启动手柄"):
            robot_data_manager.read_from_memory()
            robot_data_manager.data.ros_action["joy"] = RosAction.START
            robot_data_manager.update()
    with col42:
        joy_placeholder = st.empty()

    st.markdown("### 建图")
    col51, col52 = st.columns([1, 4])
    with col51:
        if st.button("启动建图"):
            robot_data_manager.read_from_memory()
            robot_data_manager.data.ros_action["cartographer"] = RosAction.START
            robot_data_manager.update()
    with col52:
        cartographer_placeholder = st.empty()

    st.markdown("### 定位")
    col61, col62 = st.columns([1, 4])
    with col61:
        if st.button("启动定位"):
            robot_data_manager.read_from_memory()
            robot_data_manager.data.ros_action["cartographer_pose"] = RosAction.START
            robot_data_manager.update()
    with col62:
        cartographer_pose_placeholder = st.empty()

if page == "运 行 监 控":
    st.title("Omni Robot 运行监控")
    st.markdown("### 控制状态")
    monitor_state_placeholder = st.empty()
    st.markdown("### 位姿状态")
    monitor_position_placeholder = st.empty()
    st.markdown(" ")
    st.markdown(" ")
    st.markdown(" ")
    st.markdown(" ")
    st.markdown(" ")
    st.markdown(" ")
    st.markdown(" ")
    st.markdown(" ")
    st.markdown(" ")
    st.markdown(" ")
    st.markdown(" ")
    st.markdown(" ")
    st.markdown(" ")
    st.markdown(" ")
    st.markdown(" ")
    st.markdown(" ")
    st.markdown(" ")
    st.markdown(" ")
    st.markdown(" ")

while page == "系 统 控 制":
    robot_data_manager.read_from_memory()
    with sensors_state_placeholder.container():
        if str(robot_data_manager.data.ros_state['sensors']) == str(RosState.INITIALIZING):
            st.warning("启动中")
        if str(robot_data_manager.data.ros_state['sensors']) == str(RosState.ERROR):
            st.error("启动错误")
        if str(robot_data_manager.data.ros_state['sensors']) == str(RosState.UNKNOWN):
            st.warning("未知")
        if str(robot_data_manager.data.ros_state['sensors']) == str(RosState.RUNNING):
            st.success("工作中")

    with chassis1_state_placeholder.container():
        if str(robot_data_manager.data.ros_state['chassis1']) == str(RosState.INITIALIZING):
            st.warning("启动中")
        if str(robot_data_manager.data.ros_state['chassis1']) == str(RosState.ERROR):
            st.error("启动错误")
        if str(robot_data_manager.data.ros_state['chassis1']) == str(RosState.UNKNOWN):
            st.warning("未知")
        if str(robot_data_manager.data.ros_state['chassis1']) == str(RosState.RUNNING):
            st.success("工作中")

    with chassis2_state_placeholder.container():
        if str(robot_data_manager.data.ros_state['chassis2']) == str(RosState.INITIALIZING):
            st.warning("启动中")
        if str(robot_data_manager.data.ros_state['chassis2']) == str(RosState.ERROR):
            st.error("启动错误")
        if str(robot_data_manager.data.ros_state['chassis2']) == str(RosState.UNKNOWN):
            st.warning("未知")
        if str(robot_data_manager.data.ros_state['chassis2']) == str(RosState.RUNNING):
            st.success("工作中")

    with joy_placeholder.container():
        if str(robot_data_manager.data.ros_state['joy']) == str(RosState.INITIALIZING):
            st.warning("启动中")
        if str(robot_data_manager.data.ros_state['joy']) == str(RosState.ERROR):
            st.error("启动错误")
        if str(robot_data_manager.data.ros_state['joy']) == str(RosState.UNKNOWN):
            st.warning("未知")
        if str(robot_data_manager.data.ros_state['joy']) == str(RosState.RUNNING):
            st.success("工作中")

    with cartographer_placeholder.container():
        if str(robot_data_manager.data.ros_state['cartographer']) == str(RosState.INITIALIZING):
            st.warning("启动中")
        if str(robot_data_manager.data.ros_state['cartographer']) == str(RosState.ERROR):
            st.error("启动错误")
        if str(robot_data_manager.data.ros_state['cartographer']) == str(RosState.UNKNOWN):
            st.warning("未知")
        if str(robot_data_manager.data.ros_state['cartographer']) == str(RosState.RUNNING):
            st.success("工作中")

    with cartographer_pose_placeholder.container():
        if str(robot_data_manager.data.ros_state['cartographer_pose']) == str(RosState.INITIALIZING):
            st.warning("启动中")
        if str(robot_data_manager.data.ros_state['cartographer_pose']) == str(RosState.ERROR):
            st.error("启动错误")
        if str(robot_data_manager.data.ros_state['cartographer_pose']) == str(RosState.UNKNOWN):
            st.warning("未知")
        if str(robot_data_manager.data.ros_state['cartographer_pose']) == str(RosState.RUNNING):
            st.success("工作中")
    # with joy_and_chassis_state_placeholder.container():
    #     if str(robot_data_manager.data.ros_state['sensors']) == str(RosState.INITIALIZING):
    #         st.warning("启动中")
    #     if str(robot_data_manager.data.ros_state['sensors']) == str(RosState.ERROR):
    #         st.error("启动错误")
    #     if str(robot_data_manager.data.ros_state['sensors']) == str(RosState.UNKNOWN):
    #         st.warning("未知")
    #     if str(robot_data_manager.data.ros_state['sensors']) == str(RosState.RUNNING):
    #         st.success("工作中")
    # time.sleep(0.1)

    time.sleep(0.1)



while page == "运 行 监 控":
    robot_data_manager.read_from_memory()

    with monitor_state_placeholder.container():
        if str(robot_data_manager.data.robot_state["control_mode"])   == str(ControlMode.STOP):
            st.error("当前控制模式: 停止")
        elif str(robot_data_manager.data.robot_state["control_mode"]) == str(ControlMode.JOY):
            st.warning("当前控制模式: 手柄")
        elif str(robot_data_manager.data.robot_state["control_mode"]) == str(ControlMode.AUTONOMOUS):
            st.success("当前控制模式: 自动")
        vx, vy, vtheta = robot_data_manager.data.robot_state["cmd_velocity"]
        st.write(f'$v_x$ = {vx:.2f},{SPACE}$v_y$ = {vy:.2f},{SPACE}$v_θ$ = {vtheta:.2f}')

    with monitor_position_placeholder.container():
        x, y, theta = robot_data_manager.data.robot_state["odom_position"]
        st.write(f'$x$ = {x:.2f},{SPACE2}$y$ = {y:.2f},{SPACE2}$θ$ = {theta:.2f}')
        vx, vy, vtheta = robot_data_manager.data.robot_state["odom_velocity"]
        st.write(f'$v_x$ = {vx:.2f},{SPACE}$v_y$ = {vy:.2f},{SPACE}$v_θ$ = {vtheta:.2f}')

    time.sleep(0.1)
