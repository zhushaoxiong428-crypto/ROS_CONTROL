import sys
if sys.prefix == '/home/wrj/.espressif/python_env/idf5.5_py3.10_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wrj/leap_ros_mcu_driver-main/managed_components/micro_ros_espidf_component/micro_ros_dev/install/ament_pycodestyle'
