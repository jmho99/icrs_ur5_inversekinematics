# ros2의 주요 인터페이스 예제

## 1. Topic

* __CPP__

    * Publisher

        ros2 run topic_ur5ik target_pub

    * subscriber

        ros2 run topic_ur5ik calc_sub

* __Python__

    * Publisher

        ros2 run topic_ur5ik target_pub_py.py


    * subscriber

        ros2 run topic_ur5ik calc_sub_py.py


## 2. Service

* __CPP__

    * Server

        ros2 run service_ur5ik calc_server

    * Client

        ros2 run service_ur5ik target_client 0.0 0.6 0.3 3.1415 0.0 0.0

        [ 뒤에 인수 입력해주기 {x,y,z,roll,pitch,yaw} ]

* __Python__

    * Server

        ros2 run service_ur5ik calc_server_py.py

    * Client

        ros2 run service_ur5ik target_client_py.py

## 3. Action

* __CPP__

    * Server

        ros2 run action_ur5ik calc_act_server

    * Client

        ros2 run action_ur5ik target_act_client

    * Using Terminal

        ros2 action send_goal /Goal_pose interfaces_ur5ik/action/CalcTheta "{ act_target: [0.0, 0.6, 0.3, 3.1415, 0.0, 0.0] }"

* __Python__

    * Server

        ros2 run action_ur5ik calc_act_server_py.py

    * Client

        ros2 run action_ur5ik target_act_client_py.py

---

## 개념 작성 페이지

노션 페이지 : <https://playful-butterkase-c7d.notion.site/c226e3ee97164fd892db570835e95e5d?pvs=4>

---

## 참고 자료
ROS2 Humble Docs : <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html#>
