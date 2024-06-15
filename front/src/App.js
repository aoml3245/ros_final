import React, { useRef, useEffect, useState } from 'react';
import Webcam from 'react-webcam';
import * as faceapi from 'face-api.js';
import axios from 'axios';
import './App.css';  // 추가적인 스타일링을 위한 CSS 파일

const App = () => {
  const webcamRef = useRef(null);
  const [detections, setDetections] = useState([]);
  const [modelsLoaded, setModelsLoaded] = useState(false);
  const [url, setUrl] = useState('');
  const [urlSet, setUrlSet] = useState(false);
  const [modalMessage, setModalMessage] = useState('Enter URL to start');

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
      if (urlSet && modelsLoaded) {
        detectFaces();
      }
    }, 1000); // 1초 간격으로 얼굴 감지
    return () => clearInterval(interval);
  }, [urlSet, modelsLoaded]);

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

  const sendMoveDirection = async (direction) => {
    await axios.post('/api/movedir', { direction: 0 });
    await axios.post('/api/movedir', { direction });
  };

  const handleSetUrl = () => {
    axios.post('/api/seturl', { url })
      .then(response => {
        setUrlSet(true);
        setModalMessage('Loading models...');
      })
      .catch(error => {
        setModalMessage('Failed to set URL. Try again.');
      });
  };

  return (
    <div style={{ position: 'relative', width: '100%', height: '100vh', textAlign: 'center' }}>
      <Webcam
        audio={false}
        ref={webcamRef}
        screenshotFormat="image/jpeg"
        videoConstraints={{ facingMode: 'user' }}
        style={{ width: '100%', height: 'auto' }}
      />
      {!(urlSet && modelsLoaded) && (
        <div className="modal">
          <div className="modal-content">
            <h2>{modalMessage}</h2>
            {!urlSet && (
              <>
                <input
                  type="text"
                  value={url}
                  onChange={(e) => setUrl(e.target.value)}
                  placeholder="Enter URL"
                  style={{ width: '80%', padding: '10px', marginBottom: '20px' }}
                />
                <button onClick={handleSetUrl} style={{ padding: '10px 20px' }}>Set URL</button>
              </>
            )}
          </div>
        </div>
      )}
    </div>
  );
};

export default App;

