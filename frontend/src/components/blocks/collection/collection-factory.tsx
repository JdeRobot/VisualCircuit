import fileStructureData from '../../../VisualCircuit-resources/block-library/file_structure.json';

/**
 * Interface for collection block data.
 */
export type CollectionBlockType = {
    [k: string]: {
        label: string;
        children?: CollectionBlockType;
    }
}

/**
 * All the blocks present in the repository.
 * This is used by Menu bar to show the block buttons.
 */
export const collectionBlocks: { 'blocks': CollectionBlockType, 
                                 'processing': CollectionBlockType,
                                 'drivers': CollectionBlockType,
                                 'library': CollectionBlockType } = {
    'blocks': {
        'control': {
            'label': 'Control',
            'children': {
                'motorDriver': {'label': 'MotorDriver'},
                'pid': {'label': 'PID'},
                'teleoperator': {'label': 'Teleoperator'}
            }
        },
        'opencv': {
            'label': 'OpenCV',
            'children': {
                'blur': { 'label': 'Blur' },
                'camera': { 'label': 'Camera' },
                'colorFilter': {'label': 'Color Filter'},
                'contourDetector': {'label': 'Contour Detector'},
                'cropper': {'label': 'Cropper'},
                'dilation': {'label': 'Dilation'},
                'edgeDetector': {'label': 'Edge Detector'},
                'erosion': {'label': 'Erosion'},
                'faceDetector': {'label': 'Face Detector'},
                'imageRead': {'label': 'Image Read'},
                'screen': { 'label': 'Screen' },
                'threshold': {'label': 'Threshold'},
                'videoStreamer': {'label': 'Video Streamer'}
            }
        },
        'rossensors': {
            'label': 'ROS-Sensors',
            'children': {
                'cameraRos': {'label': 'CameraROS'},
                'odometer': {'label': 'Odometer'},
                'imu': {'label': 'IMU'}
            }
        },
        'tensorflow': {
            'label': 'TensorFlow',
            'children': {
                'objectDetector': {'label': 'Object Detector'}
            }
        }
    },
    'processing': {
        'control': {
            'label': 'Control',
            'children': {
                'pid': {'label': 'PID'},
            }
        },
        'opencv': {
            'label': 'OpenCV',
            'children': {
                'blur': { 'label': 'Blur' },
                'colorFilter': {'label': 'Color Filter'},
                'contourDetector': {'label': 'Contour Detector'},
                'cropper': {'label': 'Cropper'},
                'dilation': {'label': 'Dilation'},
                'edgeDetector': {'label': 'Edge Detector'},
                'erosion': {'label': 'Erosion'},
                'faceDetector': {'label': 'Face Detector'},
                'threshold': {'label': 'Threshold'},
            }
        },
        'tensorflow': {
            'label': 'TensorFlow',
            'children': {
                'objectDetector': {'label': 'Object Detector'}
            }
        }
    },
    'drivers': {
        'control': {
            'label': 'Control',
            'children': {
                'motorDriver': {'label': 'MotorDriver'},
                'motorDriverRos2': {'label': 'MotorDriverROS2'},
                'teleoperator': {'label': 'Teleoperator'}
            }
        },
        'opencv': {
            'label': 'OpenCV',
            'children': {
                'camera': { 'label': 'Camera' },
                'imageRead': {'label': 'Image Read'},
                'screen': { 'label': 'Screen' },
                'videoStreamer': {'label': 'Video Streamer'}
            }
        },
        'rossensors': {
            'label': 'ROS-Sensors',
            'children': {
                'cameraRos': {'label': 'CameraROS'},
                'odometer': {'label': 'Odometer'},
                'imu': {'label': 'IMU'}
            }
        },
        'ros2sensors': {
            'label': 'ROS2-Sensors',
            'children': {
                'cameraRos2': {'label': 'CameraROS2'},
                'laserRos2': {'label': 'LaserROS2'},
            }
        },
    },

    'library': fileStructureData
}

/**
 * Import the data of specified block
 * @param name Name / type of block
 * @returns Imported json of the specified block
 */
export function getCollectionBlock(name: string) {

    switch (name) {
        case 'drivers.control.motorDriver':
            return import('./control/MotorDriver.json');
            // return import('../../../VisualCircuit-resources/block-library/Controllers/MotorDriver.json');
        case 'drivers.control.motorDriverRos2':
            return import('./control/MotorDriverROS2.json');
        case 'processing.control.pid':
            return import('./control/PID.json');
        case 'drivers.control.teleoperator':
            return import('./control/Teleoperator.json');
        case 'processing.opencv.blur':
            return import('./opencv/Blur.json');
        case 'drivers.opencv.camera':
            return import('./opencv/Camera.json');
        case 'processing.opencv.colorFilter':
            return import('./opencv/ColorFilter.json');
        case 'processing.opencv.contourDetector':
            return import('./opencv/ContourDetector.json');
        case 'processing.opencv.cropper':
            return import('./opencv/Cropper.json');
        case 'processing.opencv.dilation':
            return import('./opencv/Dilation.json');
        case 'processing.opencv.edgeDetector':
            return import('./opencv/EdgeDetector.json');
        case 'processing.opencv.erosion':
            return import('./opencv/Erosion.json');
        case 'processing.opencv.faceDetector':
            return import('./opencv/FaceDetector.json');
        case 'drivers.opencv.imageRead':
            return import('./opencv/ImageRead.json');
        case 'drivers.opencv.screen':
            return import('./opencv/Screen.json');
        case 'processing.opencv.threshold':
            return import('./opencv/Threshold.json');
        case 'drivers.opencv.videoStreamer':
            return import('./opencv/VideoStreamer.json');
        case 'drivers.rossensors.cameraRos':
            return import('./ros-sensors/ROSCamera.json');
        case 'drivers.rossensors.odometer':
            return import('./ros-sensors/Odometer.json');
        case 'drivers.rossensors.imu':
            return import('./ros-sensors/IMU.json');
        case 'drivers.ros2sensors.cameraRos2':
            return import('./ros-sensors/ROS2Camera.json');
        case 'drivers.ros2sensors.laserRos2':
            return import('./ros-sensors/ROS2LaserScan.json');
        case 'processing.tensorflow.objectDetector':
            return import('./tensorflow/ObjectDetector.json');
        
        default:
            break;
    }
}
