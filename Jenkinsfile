pipeline {
    agent {
         dockerfile true 
    }
    stages {
        stage('Init') {
            steps {
                script {
                    notifyBitbucket()
                }
                sh 'rm -rf build log install'
            }
        }
        stage('Build') {
            steps {
                sh '. /opt/ros/foxy/setup.sh && colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCHECK_TIDY=ON'
            }
        }
        stage('Test') {
            steps {
                sh '. /opt/ros/foxy/setup.sh && . install/setup.sh && colcon test && colcon test-result'    
            }
        }
    }
    post {
        always {
            script {
                notifyBitbucket()
            }
        }
    }
}
