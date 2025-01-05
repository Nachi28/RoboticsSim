import React, { useState, useEffect } from 'react';
import { Card } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Label } from '@/components/ui/label';
import { RadioGroup, RadioGroupItem } from '@/components/ui/radio-group';
import { Slider } from '@/components/ui/slider';
import { AlertDialog, AlertDialogContent, AlertDialogDescription, AlertDialogHeader, AlertDialogTitle, AlertDialogTrigger, AlertDialogFooter, AlertDialogCancel } from "@/components/ui/alert-dialog";

const DEFAULT_JOINT = { angle: 45, min: -120, max: 120, length: 80 };
const BASE_X = 350;
const BASE_Y = 350;
const FLOOR_Y = BASE_Y;

const checkFloorCollision = (jointConfig) => {
  let currentX = BASE_X;
  let currentY = BASE_Y;
  let currentAngle = 0;

  for (const joint of jointConfig) {
    currentAngle += joint.angle;
    const dx = joint.length * Math.cos((currentAngle * Math.PI) / 180);
    const dy = -joint.length * Math.sin((currentAngle * Math.PI) / 180);
    currentY += dy;

    if (currentY > FLOOR_Y) {
      return false;
    }
  }
  return true;
};

const calculateForwardKinematics = (jointAngles) => {
  let x = BASE_X;
  let y = BASE_Y;
  let currentAngle = 0;

  jointAngles.forEach((joint) => {
    currentAngle += joint.angle;
    x += joint.length * Math.cos((currentAngle * Math.PI) / 180);
    y -= joint.length * Math.sin((currentAngle * Math.PI) / 180);
  });

  // Return coordinates relative to base
  return {
    x: Math.round(x - BASE_X),
    y: Math.round(BASE_Y - y)
  };
};


const RobotKinematicsPlayground = () => {
  // Add new state variables
  const [ikParams, setIkParams] = useState({
    numAttempts: 20,
    maxIterations: 100,
    tolerance: 0.1
  });

  const [mode, setMode] = useState(() => {
    const saved = localStorage.getItem('robotMode');
    return saved || 'forward';
  });
  const [showModeSelect, setShowModeSelect] = useState(true);

  const [numJoints, setNumJoints] = useState(() => {
    const saved = localStorage.getItem('robotNumJoints');
    return saved ? parseInt(saved) : 2;
  });

  const [joints, setJoints] = useState(() => {
    const saved = localStorage.getItem('robotJoints');
    return saved ? JSON.parse(saved) : Array(2).fill(DEFAULT_JOINT);
  });

  const [endEffector, setEndEffector] = useState(() => {
    const saved = localStorage.getItem('robotEndEffector');
    return saved ? JSON.parse(saved) : calculateForwardKinematics(Array(2).fill(DEFAULT_JOINT));

  });

  const [possibleSolutions, setPossibleSolutions] = useState([]);
  const [selectedSolution, setSelectedSolution] = useState(0);

  useEffect(() => {
    localStorage.setItem('robotMode', mode);
    localStorage.setItem('robotJoints', JSON.stringify(joints));
    localStorage.setItem('robotEndEffector', JSON.stringify(endEffector));
    localStorage.setItem('robotNumJoints', numJoints.toString());
  }, [mode, joints, endEffector, numJoints]);

  useEffect(() => {
    if (joints.length !== numJoints) {
      const newJoints = Array(numJoints).fill(DEFAULT_JOINT);
      joints.forEach((joint, i) => {
        if (i < numJoints) newJoints[i] = joint;
      });
      setJoints(newJoints);
      // Update end effector position when joints change
      setEndEffector(calculateForwardKinematics(newJoints));
    }
  }, [numJoints]);

  useEffect(() => {
    if (mode === 'inverse') {
      const absoluteX = BASE_X + endEffector.x;
      const absoluteY = BASE_Y - endEffector.y;
      const solutions = calculateInverseKinematics(absoluteX, absoluteY, joints);
      setPossibleSolutions(solutions);
      if (solutions.length > 0) {
        setJoints(solutions[0]);
      }
    }
  }, [mode, numJoints]);



  // Modified to handle multiple joints using iterative optimization
  const calculateInverseKinematics = (targetX, targetY, currentJoints) => {
    const solutions = [];
    const numAttempts = 20;
    const maxIterations = 100;
    const tolerance = 0.1;

    // Helper function to calculate end effector position
    const calculateEndPoint = (angles) => {
      let x = BASE_X;
      let y = BASE_Y;
      let angle = 0;

      for (let i = 0; i < angles.length; i++) {
        angle += angles[i];
        x += currentJoints[i].length * Math.cos((angle * Math.PI) / 180);
        y -= currentJoints[i].length * Math.sin((angle * Math.PI) / 180);
      }

      return { x, y };
    };

    // Try multiple initial configurations
    for (let attempt = 0; attempt < numAttempts; attempt++) {
      let angles = currentJoints.map(joint => {
        // Generate different initial angles for each attempt
        const randomOffset = (Math.random() - 0.5) * 60;
        return { ...joint, angle: joint.angle + randomOffset };
      });

      for (let iter = 0; iter < maxIterations; iter++) {
        const endPoint = calculateEndPoint(angles.map(j => j.angle));
        const dx = targetX - endPoint.x;
        const dy = targetY - endPoint.y;

        if (Math.sqrt(dx * dx + dy * dy) < tolerance) {
          // Solution found
          if (checkFloorCollision(angles)) {
            solutions.push(angles);
          }
          break;
        }

        // Jacobian approximation and angle updates
        for (let i = 0; i < angles.length; i++) {
          const delta = 0.1;
          angles[i].angle += delta;
          const endPointPlus = calculateEndPoint(angles.map(j => j.angle));
          angles[i].angle -= delta;

          const jx = (endPointPlus.x - endPoint.x) / delta;
          const jy = (endPointPlus.y - endPoint.y) / delta;

          const adjustment = (dx * jx + dy * jy) / (jx * jx + jy * jy) * 0.5;
          const newAngle = angles[i].angle + adjustment;

          // Apply joint limits
          angles[i].angle = Math.max(angles[i].min, Math.min(angles[i].max, newAngle));
        }
      }
    }

    return solutions;
  };

  const handleAngleChange = (index, newAngle) => {
    const newJoints = [...joints];
    const testJoints = [...joints];
    testJoints[index] = { ...joints[index], angle: newAngle };

    newAngle = Math.min(Math.max(newAngle, joints[index].min), joints[index].max);

    if (checkFloorCollision(testJoints)) {
      newJoints[index] = { ...joints[index], angle: newAngle };
      setJoints(newJoints);

      if (mode === 'forward') {
        setEndEffector(calculateForwardKinematics(newJoints));
      }
    }
  };

  const handleEndEffectorChange = (newX, newY) => {
    if (newY === undefined) newY = endEffector.y;
    if (newX === undefined) newX = endEffector.x;

    // Convert relative coordinates to absolute for calculations
    var absoluteX = BASE_X + newX;
    var absoluteY = BASE_Y - newY;

    // Constrain Y to be above floor
    if (absoluteY > FLOOR_Y) absoluteY = FLOOR_Y;

    const maxReach = joints.reduce((sum, joint) => sum + joint.length, 0);
    const distanceFromBase = Math.sqrt(Math.pow(absoluteX - BASE_X, 2) + Math.pow(absoluteY - BASE_Y, 2));

    // Constrain to maximum reach
    if (distanceFromBase > maxReach) {
      const angle = Math.atan2(absoluteY - BASE_Y, absoluteX - BASE_X);
      absoluteX = BASE_X + maxReach * Math.cos(angle);
      absoluteY = BASE_Y + maxReach * Math.sin(angle);
    }

    // Store relative coordinates in state
    const newEndEffector = {
      x: Math.round(absoluteX - BASE_X),
      y: Math.round(BASE_Y - absoluteY)
    };
    setEndEffector(newEndEffector);

    const solutions = calculateInverseKinematics(absoluteX, absoluteY, joints);
    setPossibleSolutions(solutions);

    if (solutions.length > 0) {
      const validIndex = Math.min(selectedSolution, solutions.length - 1);
      setSelectedSolution(validIndex);
      setJoints(solutions[validIndex]);
    }
  };

  const handleReset = () => {
    const defaultJoints = Array(2).fill({ ...DEFAULT_JOINT });
    setNumJoints(2);
    setJoints(defaultJoints);
    setEndEffector(calculateForwardKinematics(defaultJoints));
    setPossibleSolutions([]);
    setSelectedSolution(0);
  };

  const getHelpContent = (mode) => {
    if (!mode || mode === 'initial') {
      return {
        title: "Welcome to Robotics Sim 2D",
        description: "Choose between two modes:\n\nForward Kinematics: Control joint angles directly to move the robot arm.\n\nInverse Kinematics: Specify target position and let the system calculate joint angles."
      };
    }
    return mode === 'forward' ? {
      title: "Forward Kinematics Mode",
      description: "Instructions:\n1. Adjust joint angles using sliders\n2. Watch end-effector position update\n3. Observe joint limits\n4. Floor collision detection active\n5. Reset to default position anytime"
    } : {
      title: "Inverse Kinematics Mode",
      description: "Instructions:\n1. Input desired end-effector position\n2. System calculates possible joint configurations\n3. Choose between multiple solutions\n4. Adjust IK parameters for better solutions\n5. Floor collision detection active"
    };
  };


  const getRobotPath = (jointConfig = joints) => {
    if (!Array.isArray(jointConfig) || jointConfig.some((joint) => !joint)) {
      console.warn("Invalid joint configuration:", jointConfig);
      return ""; // Return an empty path if the config is invalid
    }

    let path = `M ${BASE_X},${FLOOR_Y} `;
    let currentX = BASE_X;
    let currentY = FLOOR_Y;
    let currentAngle = 0;

    jointConfig.forEach(joint => {
      currentAngle += joint.angle;
      const dx = joint.length * Math.cos((currentAngle * Math.PI) / 180);
      const dy = -joint.length * Math.sin((currentAngle * Math.PI) / 180);
      currentX += dx;
      currentY += dy;
      path += `L ${currentX},${currentY} `;
    });

    return path;
  };

  return (
    <div className="flex flex-col min-h-screen">
      {/* Navbar */}
      <div className="w-full bg-slate-900 text-white p-4 flex items-center justify-between">
        <div className="text-xl font-bold cursor-pointer hover:text-blue-200 hover:scale-105 transition-all" onClick={() => {
          setShowModeSelect(true);
        }}
        >Robotics Sim 2D</div>
        {!showModeSelect && (
          <div className="flex gap-0 md:gap-4 lg:gap-6">
            <Button variant="ghost" onClick={handleReset}>Reset</Button>
            <Button variant="ghost" onClick={() => setShowModeSelect(true)}>Change Mode</Button>
            <AlertDialog>
              <AlertDialogTrigger asChild>
                <Button variant="ghost">Help</Button>
              </AlertDialogTrigger>
              <AlertDialogContent>
                <AlertDialogHeader>
                  <AlertDialogTitle>{getHelpContent(mode).title}</AlertDialogTitle>
                  <AlertDialogDescription className="whitespace-pre-line">
                    {getHelpContent(mode).description}
                  </AlertDialogDescription>
                </AlertDialogHeader>
                <AlertDialogFooter>
                  <AlertDialogCancel>Close</AlertDialogCancel>
                </AlertDialogFooter>
              </AlertDialogContent>
            </AlertDialog>
          </div>
        )}
      </div>

      {/* Main Content */}
      <div style={{ backgroundColor: 'lavender' }} className="flex-1 w-full p-4">
        {showModeSelect ? (
          <div className="w-full h-[85vh] flex items-center justify-center">
            <Card className="p-8 max-w-md w-full shadow-lg rounded-lg">
              <h2 className="text-3xl font-bold mb-6 text-center">Robotics Sim 2D</h2>
              <h3 className="text-xl mb-6 text-center">Select Kinematics Mode</h3>
              <div className="flex gap-6">
                <Button
                  className="flex-1 text-xl py-4"
                  onClick={() => {
                    setMode('forward');
                    setShowModeSelect(false);
                  }}
                >
                  Forward
                </Button>
                <Button
                  className="flex-1 text-xl py-4"
                  onClick={() => {
                    setMode('inverse');
                    setShowModeSelect(false);
                  }}
                >
                  Inverse
                </Button>
              </div>
            </Card>
          </div>
        ) : (
          <div className="flex flex-col-reverse md:flex-row gap-4 min-h-0">

            {/* Controls Panel */}
            <Card className="p-6 h-full max-h-[85vh] max-w-96 flex flex-col">
              <div className="flex justify-between mb-4">
                <h2 className="text-xl font-bold">
                  {mode === 'forward' ? 'Forward' : 'Inverse'} Kinematics
                </h2>
                <div className="flex gap-2"></div>
              </div>


              <div className="flex-grow overflow-y-auto p-2">
                <div className="mb-4">
                  <Label>Number of Joints (2-6)</Label>
                  <Input
                    type="number"
                    min="2"
                    max="6"
                    value={numJoints}
                    onChange={(e) => setNumJoints(Math.min(6, Math.max(2, parseInt(e.target.value) || 2)))}
                  />
                </div>

                {/* End Effector Position - Highlighted for Forward Mode */}
                <div className={`mb-4 rounded-lg ${mode === 'forward' ? 'bg-blue-50 border border-blue-200' : ''}`}>
                  <Label>End Effector Position</Label>
                  <div className="grid grid-cols-2 gap-2">
                    <div>
                      <Label>X Position</Label>
                      <Input
                        type="number"
                        value={Math.round(endEffector.x)}
                        onChange={(e) => handleEndEffectorChange(parseFloat(e.target.value), undefined)}
                        disabled={mode === 'forward'}
                      />
                    </div>
                    <div>
                      <Label>Y Position</Label>
                      <Input
                        type="number"
                        value={Math.round(endEffector.y)}
                        onChange={(e) => handleEndEffectorChange(undefined, parseFloat(e.target.value))}
                        disabled={mode === 'forward'}
                      />
                    </div>
                  </div>
                </div>

                {/* Select Solution */}
                {mode === 'inverse' && possibleSolutions.length > 1 && (
                  <div className="mb-4">
                    <Label>Solution Variant ({possibleSolutions.length} available)</Label>
                    <Input
                      type="number"
                      min="0"
                      max={possibleSolutions.length - 1}
                      value={selectedSolution}
                      onChange={(e) => {
                        const newIndex = parseInt(e.target.value) || 0;
                        if (newIndex >= 0 && newIndex < possibleSolutions.length) {
                          setSelectedSolution(newIndex);
                          setJoints(possibleSolutions[newIndex]);
                        }
                      }}
                    />
                  </div>
                )}


                {/* IK Parameters - Show only in inverse mode */}
                {mode === 'inverse' && (
                  <div className="mb-4">
                    <Label>IK Parameters</Label>
                    <div className="grid grid-cols-1 gap-2">
                      <div>
                        <Label>Number of Attempts</Label>
                        <Input
                          type="number"
                          value={ikParams.numAttempts}
                          onChange={(e) => setIkParams({ ...ikParams, numAttempts: parseInt(e.target.value) })}
                        />
                      </div>
                      <div>
                        <Label>Max Iterations</Label>
                        <Input
                          type="number"
                          value={ikParams.maxIterations}
                          onChange={(e) => setIkParams({ ...ikParams, maxIterations: parseInt(e.target.value) })}
                        />
                      </div>
                      <div>
                        <Label>Tolerance</Label>
                        <Input
                          type="number"
                          step="0.01"
                          value={ikParams.tolerance}
                          onChange={(e) => setIkParams({ ...ikParams, tolerance: parseFloat(e.target.value) })}
                        />
                      </div>
                    </div>
                  </div>
                )}

                {/* Joint Controls - Highlighted for Inverse Mode */}
                {joints.map((joint, index) => (
                  <div key={index} className={`mb-4 p-2 rounded-lg ${mode === 'inverse' ? 'bg-blue-50 border border-blue-200' : ''}`}>
                    <Label>Joint {index + 1} Angle: {Math.round(joint.angle)}°</Label>
                    <Slider
                      value={[joint.angle]}
                      min={joint.min}
                      max={joint.max}
                      step={1}
                      onValueChange={([value]) => handleAngleChange(index, value)}
                      disabled={mode === 'inverse'}
                      className="my-2"
                    />
                    <div className="grid grid-cols-2 gap-2 mb-2">
                      <div>
                        <Label>Min Angle</Label>
                        <Input
                          type="number"
                          value={joint.min}
                          onChange={(e) => {
                            const newJoints = [...joints];
                            newJoints[index] = { ...joint, min: parseInt(e.target.value) };
                            setJoints(newJoints);
                          }}
                        />
                      </div>
                      <div>
                        <Label>Max Angle</Label>
                        <Input
                          type="number"
                          value={joint.max}
                          onChange={(e) => {
                            const newJoints = [...joints];
                            newJoints[index] = { ...joint, max: parseInt(e.target.value) };
                            setJoints(newJoints);
                          }}
                        />
                      </div>
                    </div>
                  </div>
                ))}



              </div>

            </Card>

            <div className="flex-grow ">
              <Card className="w-full h-full max-h-[85vh] p-4">
                <svg viewBox='0 0 700 400' className="w-full h-full">
                  <line x1="0" y1={FLOOR_Y} x2="700" y2={FLOOR_Y} stroke="black" strokeWidth="2" />
                  {mode === 'inverse' && possibleSolutions.length > 0 ? (
                    // Render all possible solutions first (unselected ones)
                    possibleSolutions.map((solution, index) => (
                      <path
                        key={index}
                        d={getRobotPath(solution)}
                        stroke={index === selectedSolution ? "blue" : "rgba(0,0,255,0.1)"}
                        strokeWidth="2"
                        fill="none"
                        style={{ cursor: "pointer" }}
                        onClick={() => {
                          setSelectedSolution(index);
                          setJoints(possibleSolutions[index]);
                        }}
                      />
                    ))
                  ) : (
                    <path
                      d={getRobotPath(joints)}
                      stroke="blue"
                      strokeWidth="2"
                      fill="none"
                    />
                  )}

                  {/* Base point */}
                  <circle cx={BASE_X} cy={FLOOR_Y} r="3" fill="grey" />

                  {/* Joints and End Effector */}
                  {(mode === 'inverse' ? (possibleSolutions[selectedSolution] || joints) : joints).reduce((points, joint, index) => {
                    const prevPoint = index === 0 ? { x: BASE_X, y: FLOOR_Y } : points[index - 1];
                    const angle = (mode === 'inverse' ?
                      (possibleSolutions[selectedSolution] || joints) :
                      joints
                    ).slice(0, index + 1).reduce((sum, j) => sum + j.angle, 0);
                    const x = prevPoint.x + joint.length * Math.cos((angle * Math.PI) / 180);
                    const y = prevPoint.y - joint.length * Math.sin((angle * Math.PI) / 180);
                    points.push({ x, y });
                    return points;
                  }, []).map((point, index, arr) => (
                    <g
                      key={index}>
                      <circle
                        cx={point.x}
                        cy={point.y}
                        r="2.5"
                        fill="red"
                      />
                      {index === arr.length - 1 && (
                        <>
                          <circle
                            cx={point.x}
                            cy={point.y}
                            r="6"
                            fill="none"
                            stroke="red"
                            strokeWidth="1"
                          />
                          <circle
                            cx={point.x}
                            cy={point.y}
                            r="10"
                            fill="none"
                            stroke="red"
                            strokeWidth="1"
                            strokeDasharray="2,2"
                          />
                        </>
                      )}
                    </g>

                  ))}
                </svg>
              </Card>
            </div>

          </div>
        )}
      </div>
    </div >
  );
};
export default RobotKinematicsPlayground;