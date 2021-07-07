export type CollectionBlockType = {
    [k: string]: {
        label: string;
        children?: CollectionBlockType;
    }
}

export const collectionBlocks: { 'blocks': CollectionBlockType } = {
    'blocks': {
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
        'tensorflow': {
            'label': 'TensorFlow',
            'children': {
                'objectDetector': {'label': 'Object Detector'}
            }
        }
    }
}


export function getCollectionBlock(name: string) {

    switch (name) {
        case 'blocks.opencv.blur':
            return import('./opencv/Blur.json');
        case 'blocks.opencv.camera':
            return import('./opencv/Camera.json');
        case 'blocks.opencv.colorFilter':
            return import('./opencv/ColorFilter.json');
        case 'blocks.opencv.contourDetector':
            return import('./opencv/ContourDetector.json');
        case 'blocks.opencv.cropper':
            return import('./opencv/Cropper.json');
        case 'blocks.opencv.dilation':
            return import('./opencv/Dilation.json');
        case 'blocks.opencv.edgeDetector':
            return import('./opencv/EdgeDetector.json');
        case 'blocks.opencv.erosion':
            return import('./opencv/Erosion.json');
        case 'blocks.opencv.faceDetector':
            return import('./opencv/FaceDetector.json');
        case 'blocks.opencv.imageRead':
            return import('./opencv/ImageRead.json');
        case 'blocks.opencv.screen':
            return import('./opencv/Screen.json');
        case 'blocks.opencv.threshold':
            return import('./opencv/Threshold.json');
        case 'blocks.opencv.videoStreamer':
            return import('./opencv/VideoStreamer.json');
        case 'blocks.tensorflow.objectDetector':
            return import('./tensorflow/ObjectDetector.json');
        default:
            break;
    }
}