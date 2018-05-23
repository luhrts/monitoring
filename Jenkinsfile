node {

  pythonPath = "PYTHONPATH=\$PYTHONPATH:/usr/lib/python2.7/dist-packages/"
  def err = null
  currentBuild.result = "SUCCESS"

  try {
    stage('Clean old Build'){
        dir('catkin_ws/src') {
            deleteDir()
        }
    }
  
    stage('Checkout'){
      checkout scm
    }

    stage('ROS-Build'){
         dir ('catkin_ws/src') {
            withEnv([pythonPath]) {
                sh 'source /opt/ros/kinetic/setup.bash; catkin_init_workspace'
            }
        }
      dir ('catkin_ws') { 
        withEnv([pythonPath]) {
          sh 'source /opt/ros/kinetic/setup.bash; catkin_make'
        }
      }
    }

  } catch (caughtError) {
    err = caughtError
    currentBuild.result = "FAILURE"
    step([$class: 'Mailer', notifyEveryUnstableBuild: true, recipients: 'carstensen@rts.uni-hannover.de', sendToIndividuals: true])
  } finally {
    if (err) {
        throw err
    }
  }

}

