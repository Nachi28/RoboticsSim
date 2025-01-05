// App.jsx
import React from 'react';
import RobotKinematicsPlayground from './components/RobotKinematicsPlayground';
import './global.css';

function App() {
  return (
    <div className="min-h-screen bg-background text-foreground">
      <main >
        <RobotKinematicsPlayground />
      </main>
    </div>
  );
}

export default App;