// import fileStructureData from '../../../VisualCircuit-resources/block-library/file_structure.json';

import fileStructureData from '../VisualCircuit-resources/block-library/file_structure.json';

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
export const collectionBlocks: {
    'blocks': CollectionBlockType,
    'processing': CollectionBlockType,
    'drivers': CollectionBlockType,
    'library': CollectionBlockType
} = {
    'blocks': {
        'control': {
            'label': 'Control',
            'children': {
                'MotorDriver': { 'label': 'MotorDriver' },
                'PID': { 'label': 'PID' },
                'Teleoperator': { 'label': 'Teleoperator' }
            }
        },
        'opencv': {
            'label': 'OpenCV',
            'children': {
                'Blur': { 'label': 'Blur' },
                'Camera': { 'label': 'Camera' },
                'ColorFilter': { 'label': 'Color Filter' },
                'ContourDetector': { 'label': 'Contour Detector' },
                'Cropper': { 'label': 'Cropper' },
                'Dilation': { 'label': 'Dilation' },
                'EdgeDetector': { 'label': 'Edge Detector' },
                'Erosion': { 'label': 'Erosion' },
                'FaceDetector': { 'label': 'Face Detector' },
                'ImageRead': { 'label': 'Image Read' },
                'Screen': { 'label': 'Screen' },
                'Threshold': { 'label': 'Threshold' },
                'VideoStreamer': { 'label': 'Video Streamer' }
            }
        },
        'ros-sensors': {
            'label': 'ROS-Sensors',
            'children': {
                'CameraRos': { 'label': 'CameraROS' },
                'Odometer': { 'label': 'Odometer' },
                'IMU': { 'label': 'IMU' }
            }
        },
        'tensorflow': {
            'label': 'TensorFlow',
            'children': {
                'ObjectDetector': { 'label': 'Object Detector' }
            }
        }
    },
    'processing': {
        'control': {
            'label': 'Control',
            'children': {
                'PID': { 'label': 'PID' },
            }
        },
        'opencv': {
            'label': 'OpenCV',
            'children': {
                'Blur': { 'label': 'Blur' },
                'ColorFilter': { 'label': 'Color Filter' },
                'ContourDetector': { 'label': 'Contour Detector' },
                'Cropper': { 'label': 'Cropper' },
                'Dilation': { 'label': 'Dilation' },
                'EdgeDetector': { 'label': 'Edge Detector' },
                'Erosion': { 'label': 'Erosion' },
                'FaceDetector': { 'label': 'Face Detector' },
                'Threshold': { 'label': 'Threshold' },
            }
        },
        'tensorflow': {
            'label': 'TensorFlow',
            'children': {
                'ObjectDetector': { 'label': 'Object Detector' }
            }
        }
    },
    'drivers': {
        'control': {
            'label': 'Control',
            'children': {
                'MotorDriver': { 'label': 'MotorDriver' },
                'MotorDriverRos2': { 'label': 'MotorDriverROS2' },
                'Teleoperator': { 'label': 'Teleoperator' }
            }
        },
        'opencv': {
            'label': 'OpenCV',
            'children': {
                'Camera': { 'label': 'Camera' },
                'ImageRead': { 'label': 'Image Read' },
                'Screen': { 'label': 'Screen' },
                'VideoStreamer': { 'label': 'Video Streamer' }
            }
        },
        'ros-sensors': {
            'label': 'ROS-Sensors',
            'children': {
                'ROSCamera': { 'label': 'CameraROS' },
                'Odometer': { 'label': 'Odometer' },
                'IMU': { 'label': 'IMU' }
            }
        },
        'ros2sensors': {
            'label': 'ROS2-Sensors',
            'children': {
                'CameraROS2': { 'label': 'CameraROS2' },
                'LaserScanROS2': { 'label': 'LaserROS2' },
            }
        },
    },

    'library': fileStructureData
}


function convertToFilePath(input: string, path: string) {
    // Use regular expression to match and capture parts of the input string
    const match = input.match(/[A-Z0-9a-z-]+/g);
    console.log(match)
    // Check if a match is found
    if (match && match.length >= 2) {
        // Remove the first word until the dot
        match.shift();

        // Convert the remaining matched parts to the desired format
        const result = match.map(part => part).join('/') + '.json';

        // Add './' to the beginning of the result
        return path + result;
    } else {
        // Handle the case when the input doesn't match the expected format
        console.log("Invalid input format");
        return ""; // or throw an error, return an error message, etc.
    }
}

/**
 * Import the data of specified block
 * @param name Name / type of block
 * @returns Imported json of the specified block
 */
export function getCollectionBlock(name: string) {
    let PATH_TO_LIB = 'blocks/collection/'
    if(name.startsWith('library')){
        // PATH_TO_LIB = '../../../VisualCircuit-resources/block-library/'
        PATH_TO_LIB = 'blocks/VisualCircuit-resources/block-library/'
    }
    const output = convertToFilePath(name, PATH_TO_LIB);

    // V. V. IMP
    // The two ../ are required to ensure that the bundler loads the objects in the module that we have given
    // If we don't give the 2x ../, the bundler will not load these objects and will throw an error
    // The library blocks being loaded are dependent on this
    // link: https://webpack.js.org/api/module-methods/#dynamic-expressions-in-import
    return import('../../' + output);
}


// Old method of converting the name to file path
    /*
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
    }*/