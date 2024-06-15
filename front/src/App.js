
import React, { useRef, useEffect, useState } from 'react';
import Webcam from 'react-webcam';
import * as faceapi from 'face-api.js';
import axios from 'axios';

const App = () => {
  const webcamRef = useRef(null);
  const [detections, setDetections] = useState([]);

  const [modelsLoaded, setModelsLoaded] = React.useState(false);
  const [captureVideo, setCaptureVideo] = React.useState(false);

  const [url, setUrl] = useState('');
  const [moveCommand, setMoveCommand] = useState('');
  const [direction, setDirection] = useState(0);

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
    }, 2000);
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

  const sendMoveDirection = async (direction) => {
    await axios.post('/api/movedir', { direction: "0" });
    
    axios.post('/api/movedir', { direction })
      .then(response => {
        console.log(response.data);
        alert(`Response from /api/movedir: ${response.data.message}`);
      })
      .catch(error => {
        console.error('There was an error making the POST request!', error);
      });
  };

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

  const handleMove = () => {
    axios.post('/api/move', { command: moveCommand })
      .then(response => {
        console.log(response.data);
        alert(`Response from /api/move: ${response.data.message}`);
      })
      .catch(error => {
        console.error('There was an error making the POST request!', error);
      });
  };



  const handleMoveDir = () => {
    axios.post('/api/movedir', { direction: parseInt(direction) })
      .then(response => {
        console.log(response.data);
        alert(`Response from /api/movedir: ${response.data.message}`);
      })
      .catch(error => {
        console.error('There was an error making the POST request!', error);
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

      <div>
        <h2>Set URL</h2>
        <input type="text" value={url} onChange={(e) => setUrl(e.target.value)} placeholder="Enter URL" />
        <button onClick={handleSetUrl}>Set URL</button>
      </div>

      <div>
        <h2>Move Command</h2>
        <input type="text" value={moveCommand} onChange={(e) => setMoveCommand(e.target.value)} placeholder="Enter move command (e.g., 'X10 Y10')" />
        <button onClick={handleMove}>Send Move Command</button>
      </div>

      <div>
        <h2>Move Direction</h2>
        <input type="number" value={direction} onChange={(e) => setDirection(e.target.value)} placeholder="Enter direction (0, 1, -1)" />
        <button onClick={handleMoveDir}>Send Move Direction</button>
      </div>
    </div>
  );
};

export default App;

