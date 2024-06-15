import React, { useRef, useEffect, useState } from 'react';
import Webcam from 'react-webcam';
import * as faceapi from 'face-api.js';
import axios from 'axios';

const App = () => {
  const webcamRef = useRef(null);
  const [detections, setDetections] = useState([]);
  const [modelsLoaded, setModelsLoaded] = useState(false);
  const [url, setUrl] = useState('');

  useEffect(() => {
    const loadModels = async () => {
      const MODEL_URL = '/models';
      Promise.all([
        faceapi.nets.tinyFaceDetector.loadFromUri(MODEL_URL),
        faceapi.nets.faceLandmark68Net.loadFromUri(MODEL_URL),
        faceapi.nets.faceRecognitionNet.loadFromUri(MODEL_URL),
        faceapi.nets.faceExpressionNet.loadFromUri(MODEL_URL),
      ]).then(() => setModelsLoaded(true));
    }
    loadModels();
  }, []);

  const detectFaces = async () => {
    if (webcamRef.current && webcamRef.current.video.readyState === 4) {
      const video = webcamRef.current.video;
      const detections = await faceapi.detectAllFaces(video, new faceapi.TinyFaceDetectorOptions()).withFaceLandmarks().withFaceDescriptors();
      setDetections(detections);
    }
  };

  useEffect(() => {
    const interval = setInterval(() => {
      detectFaces();
    }, 1000); // 1초 간격으로 얼굴 감지
    return () => clearInterval(interval);
  }, []);

  useEffect(() => {
    if (detections.length > 0) {
      const detection = detections[0].detection.box;
      const faceCenterX = detection._x + detection._width / 2;
      const screenWidth = webcamRef.current.video.videoWidth;
      const leftBoundary = screenWidth / 5;
      const rightBoundary = 4 * screenWidth / 5;

      if (faceCenterX < leftBoundary) {
        sendMoveDirection(1); // 오른쪽으로 이동
      } else if (faceCenterX > rightBoundary) {
        sendMoveDirection(-1); // 왼쪽으로 이동
      }
    }
  }, [detections]);

  const sendMoveDirection = (direction) => {
    axios.post('/api/movedir', { direction })
      .then(response => {
        console.log(response.data);
        alert(`Response from /api/movedir: ${response.data.message}`);
      })
      .catch(error => {
        console.error('There was an error making the POST request!', error);
      });
  };

  const handleSetUrl = () => {
    axios.post('/api/seturl', { url })
      .then(response => {
        console.log(response.data);
        alert(`Response from /api/seturl: ${response.data.message}`);
      })
      .catch(error => {
        console.error('There was an error making the POST request!', error);
      });
  };

  return (
    <div style={{ display: 'flex', justifyContent: 'space-between', padding: '20px' }}>
      <div style={{ flex: 1 }}>
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
          {modelsLoaded && detections.map((detection, index) => (
            <div key={index}>
              <p>Face detected at: {JSON.stringify(detection.detection.box)}</p>
            </div>
          ))}
        </div>
      </div>
      <div style={{ flex: 1, display: 'flex', flexDirection: 'column', justifyContent: 'center', alignItems: 'center' }}>
        <h2>Set URL</h2>
        <input
          type="text"
          value={url}
          onChange={(e) => setUrl(e.target.value)}
          placeholder="Enter URL"
          style={{ width: '80%', padding: '10px', marginBottom: '20px' }}
        />
        <button onClick={handleSetUrl} style={{ padding: '10px 20px' }}>Set URL</button>
      </div>
    </div>
  );
};

export default App;