// src/main.jsx
import React from 'react';
import ReactDOM from 'react-dom/client';
import App from './App.jsx';
import { RosProvider } from './RosConnection.jsx'; // Import the provider
import './index.css';

ReactDOM.createRoot(document.getElementById('root')).render(
  <React.StrictMode>
    <RosProvider>
      <App />
    </RosProvider>
  </React.StrictMode>
);