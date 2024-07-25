# ros2의 주요 인터페이스 예제

## 1. Topic

* __CPP__

    * Publisher

        ros2 run calc_ur5ik_topic set_RT

    * subscriber

        ros2 run calc_ur5ik_topic ur5ik

* __Python__

    * Publisher

        ros2 run calc_ur5ik_topic set_RT_py.py


    * subscriber

        ros2 run calc_ur5ik_topic ur5ik_py.py


## 2. Service

* __CPP__

    * Server

        ros2 run calc_ur5ik_service ur5ik 0.0 0.6 0.3 3.1415 0.0 0.0

        [ 뒤에 인수 입력해주기 {x,y,z,roll,pitch,yaw} ]

    * Client

        ros2 run calc_ur5ik_service set_RT

* __Python__

    * Server

        ros2 run calc_ur5ik_service ur5ik_py.py

    * Client

        ros2 run calc_ur5ik_service set_RT_py.py

## 3. Action

* __CPP__

    * Server

        ros2 run calc_ur5ik_action ur5ik

    * Client

        ros2 run calc_ur5ik_action set_RT

    * Using Terminal

        ros2 action send_goal /Goal_pose calc_interfaces/action/CalcTheta "{ act_target: [0.0, 0.6, 0.3, 3.1415, 0.0, 0.0] }"

* __Python__

    * Server

        ros2 run calc_ur5ik_action ur5ik_py.py

    * Client

        ros2 run calc_ur5ik_action set_RT_py.py

---

## 개념 작성 페이지

노션 페이지 : <https://playful-butterkase-c7d.notion.site/c226e3ee97164fd892db570835e95e5d?pvs=4>

---

## 참고 자료
ROS2 Humble Docs : <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html#>
