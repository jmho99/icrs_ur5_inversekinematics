# ros2의 주요 인터페이스 예제

## 1. Topic

    * CPP

        * Publisher

            ros2 run calc_ur5ik_topic set_RT

        * subscriber

            ros2 run calc_ur5ik_topic ur5ik

   * Python

        * Publisher

            ros2 run calc_ur5ik_topic set_RT_py.py


        * subscriber

            ros2 run calc_ur5ik_topic ur5ik_py.py


## 2. Service

   * CPP

        * Server

            ros2 run calc_ur5ik_service ur5ik

        * Client

            ros2 run calc_ur5ik_service set_RT

   * Python

        * Server

        ros2 run calc_ur5ik_service ur5ik_py.py

        * Client

            ros2 run calc_ur5ik_service set_RT_py.py

## 3. Action

   * CPP

        * Server

            ros2 run calc_ur5ik_action ur5ik

        * Client

            ros2 run calc_ur5ik_action set_RT

   * Python

        * Server

            ros2 run calc_ur5ik_action ur5ik_py.py

        * Client

            ros2 run calc_ur5ik_action set_RT_py.py
