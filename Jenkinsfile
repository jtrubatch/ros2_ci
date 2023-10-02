pipeline {
    agent any
    stages {
        stage('BUILD'){
            steps{
                sh 'cd ~/ros2_ws/src'
                sh '''
                    #!/bin/bash
                    if [ ! -d "ros2_ci" ]; then
                        git clone https://github.com/jtrubatch/ros2_ci.git
                    else 
                        cd ros2_ci
                        git pull origin main
                    fi
                    '''
                sh 'cd ~/ros2_ws/src/ros2_ci'
                sh 'sudo docker build -f ros2.dockerfile -t ros2_ci .'
                sh 'sudo docker-compose up -d && sleep 20'
            }
        }
        stage('TEST'){
            steps{
                sh 'sudo docker exec ros2_ci bash -c "source /ros2_ws/install/setup.bash && colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+"'
            }
        }   
    }
}
// COMMENT FOR TEST PUSH FIXED BUILD COMMAND
