
import React, { useRef, useEffect, useState } from 'react';
import Webcam from 'react-webcam';
import * as faceapi from 'face-api.js';

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
    </div>
  );
};

export default App;




// function App() {

  

//   React.useEffect(() => {
//     const loadModels = async () => {
//       const MODEL_URL = '/models';

//       Promise.all([
//         faceapi.nets.tinyFaceDetector.loadFromUri(MODEL_URL),
//         faceapi.nets.faceLandmark68Net.loadFromUri(MODEL_URL),
//         faceapi.nets.faceRecognitionNet.loadFromUri(MODEL_URL),
//         faceapi.nets.faceExpressionNet.loadFromUri(MODEL_URL),
//       ]).then(setModelsLoaded(true));
//     }
//     loadModels();
//   }, []);

//   const startVideo = () => {
//     setCaptureVideo(true);
//     navigator.mediaDevices
//       .getUserMedia({ video: { width: 300 } })
//       .then(stream => {
//         let video = videoRef.current;
//         video.srcObject = stream;
//         video.play();
//       })
//       .catch(err => {
//         console.error("error:", err);
//       });
//   }

//   const handleVideoOnPlay = () => {
//     setInterval(async () => {
//       if (canvasRef && canvasRef.current) {
//         canvasRef.current.innerHTML = faceapi.createCanvasFromMedia(videoRef.current);
//         const displaySize = {
//           width: videoWidth,
//           height: videoHeight
//         }

//         faceapi.matchDimensions(canvasRef.current, displaySize);

//         const detections = await faceapi.detectAllFaces(videoRef.current, new faceapi.TinyFaceDetectorOptions()).withFaceLandmarks().withFaceExpressions();

//         const resizedDetections = faceapi.resizeResults(detections, displaySize);

//         canvasRef && canvasRef.current && canvasRef.current.getContext('2d').clearRect(0, 0, videoWidth, videoHeight);
//         canvasRef && canvasRef.current && faceapi.draw.drawDetections(canvasRef.current, resizedDetections);
//         canvasRef && canvasRef.current && faceapi.draw.drawFaceLandmarks(canvasRef.current, resizedDetections);
//         canvasRef && canvasRef.current && faceapi.draw.drawFaceExpressions(canvasRef.current, resizedDetections);
//       }
//     }, 100)
//   }


//   return (
//     <div>
//       <div style={{ textAlign: 'center', padding: '10px' }}>
//         {
//           captureVideo && modelsLoaded ?
//             <button onClick={closeWebcam} style={{ cursor: 'pointer', backgroundColor: 'green', color: 'white', padding: '15px', fontSize: '25px', border: 'none', borderRadius: '10px' }}>
//               Close Webcam
//             </button>
//             :
//             <button onClick={startVideo} style={{ cursor: 'pointer', backgroundColor: 'green', color: 'white', padding: '15px', fontSize: '25px', border: 'none', borderRadius: '10px' }}>
//               Open Webcam
//             </button>
//         }
//       </div>
//       {
//         captureVideo ?
//           modelsLoaded ?
//             <div>
//               <div style={{ display: 'flex', justifyContent: 'center', padding: '10px' }}>
//                 <video ref={videoRef} height={videoHeight} width={videoWidth} onPlay={handleVideoOnPlay} style={{ borderRadius: '10px' }} />
//                 <canvas ref={canvasRef} style={{ position: 'absolute' }} />
//               </div>
//             </div>
//             :
//             <div>loading...</div>
//           :
//           <>
//           </>
//       }
//     </div>
//   );
// }

// export default App;