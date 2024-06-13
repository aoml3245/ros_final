
import React, { useRef, useEffect, useState } from 'react';
import Webcam from 'react-webcam';
import * as faceapi from 'face-api.js';
import axios from 'axios';

const App = () => {
  const webcamRef = useRef(null);
  const [detections, setDetections] = useState([]);

  const [modelsLoaded, setModelsLoaded] = React.useState(false);
  const [captureVideo, setCaptureVideo] = React.useState(false);

  const videoRef = React.useRef();
  const videoHeight = 480;
  const videoWidth = 640;
  const canvasRef = React.useRef();

  React.useEffect(() => {
    const loadModels = async () => {
      const MODEL_URL = '/models';

      Promise.all([
        faceapi.nets.tinyFaceDetector.loadFromUri(MODEL_URL),
        faceapi.nets.faceLandmark68Net.loadFromUri(MODEL_URL),
        faceapi.nets.faceRecognitionNet.loadFromUri(MODEL_URL),
        faceapi.nets.faceExpressionNet.loadFromUri(MODEL_URL),
      ]).then(setModelsLoaded(true));
    }
    loadModels();
  }, []);

  const detectFaces = async () => {
    if (webcamRef.current && webcamRef.current.video.readyState === 4) {
      const video = webcamRef.current.video;
      // const detections =  faceapi.detectAllFaces(video, new faceapi.TinyFaceDetectorOptions()).withFaceLandmarks().withFaceDescriptors();
      const detections = await faceapi.detectAllFaces(video, new faceapi.TinyFaceDetectorOptions()).withFaceLandmarks().withFaceDescriptors();
      setDetections(detections);
    }
  };

  useEffect(() => {
    const interval = setInterval(() => {
      detectFaces();
    }, 100);
    return () => clearInterval(interval);
  }, []);

  const handleButtonClick = () => {
    axios.get('/api/hello')
      .then(response => {
        console.log(response.data);
        alert(`Response from /api/hello: ${response.data}`);
      })
      .catch(error => {
        console.error('There was an error making the GET request!', error);
      });
  };

  return (
    <div style={{ textAlign: 'center' }}>
      <h1>Face Detection</h1>
      <Webcam
        audio={false}
        ref={webcamRef}
        screenshotFormat="image/jpeg"
        videoConstraints={{ facingMode: 'user' }}
        style={{ width: '100%', height: 'auto' }}
      />
      <div>
        {!modelsLoaded && "모델 로드 중입니다."}
        {modelsLoaded ?  detections.map((detection, index) => (
           (<div key={index}>
            <p>Face detected at: {JSON.stringify(detection.detection.box)}</p>
          </div>)
        )): (<p>모델 로드 중입니다.</p>)}

        {modelsLoaded}
        
      </div>
      <button onClick={handleButtonClick}>Send GET Request to /api/hello</button>
    </div>
  );
};

export default App;

