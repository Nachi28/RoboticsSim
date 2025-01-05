
# Robotics Sim 2D

A web-based robotics simulator designed to simplify the understanding of the complex concepts of forward and inverse kinematics. The platform provides an intuitive 2D robotic arm visualization that supports 2 to 6 degrees of freedom (DoFs).

## Deployed Link
[https://roboticssim.onrender.com](https://roboticssim.onrender.com)

## Features
- **Ease of Learning**: Simplifies the complex concepts of forward and inverse kinematics through visual aids.
- **Interactive 2D Visualization**: Provides real-time feedback on a 2D robotic arm model.
- **Forward Kinematics**: Control the robotic arm directly by adjusting joint angles.
- **Inverse Kinematics**: Compute joint angles by specifying the end-effector position.
- **Support for 2-6 DoFs**: Easily configure the robotic arm with 2 to 6 joints.
- **Real-Time Collision Detection**: Detects collisions with the floor to ensure realistic simulation.
- **Multiple IK Solutions**: Offers various valid solutions for inverse kinematics.
- **Adjustable Parameters**:
  - Joint limits
  - Number of IK attempts
  - Iterations and convergence tolerance
- **Responsive Design**: Works seamlessly on various screen sizes.
- **State Persistence**: Saves configurations and progress using local storage.
- **Customizable UI**: Built with modular, reusable UI components.

## Tech Stack
- **Framework**: React
- **UI Components**: [shadcn/ui](https://ui.shadcn.com)
- **Styling**: Tailwind CSS
- **Graphics**: SVG for 2D robotic arm visualization
- **State Management**: React Hooks
- **Deployment**: Render.com

## Getting Started

### Prerequisites
- Node.js (v14.x or higher)
- npm (v6.x or higher)

### Installation
1. **Clone the repository**:
   ```bash
   git clone https://github.com/Nachi28/RoboticsSim.git
   cd RoboticsSim
   ```

2. **Install dependencies**:
   ```bash
   npm install
   ```

3. **Install required shadcn/ui components**:
   ```bash
   npx shadcn-ui@latest add card button input label slider alert-dialog radio-group
   ```

### Running the Application
1. **Start the development server**:
   ```bash
   npm run dev
   ```
2. Open [http://localhost:5173](http://localhost:5173) in your browser.

## Usage

### Forward Kinematics Mode
1. Select "Forward" mode from the initial screen.
2. Adjust joint angles using the sliders.
3. Observe the real-time update of the end-effector's position on the 2D model.
4. Configure joint parameters (e.g., limits) as required.

### Inverse Kinematics Mode
1. Select "Inverse" mode from the initial screen.
2. Input the desired X and Y coordinates for the end-effector.
3. If multiple solutions are available, choose the most appropriate one.
4. Fine-tune IK parameters for better solutions:
   - Number of attempts
   - Maximum iterations
   - Convergence tolerance

### Common Features
- Adjust the number of joints (2-6).
- Reset the simulation to default settings.
- Toggle seamlessly between modes.
- Access help documentation for guidance.

---

## Project Structure
The project is organized as follows:

```
RoboticsSim/
├── public/                  # Public assets
│   ├── favicon-32x32.png    # Favicon
│   ├── favicon.ico          # Favicon for older browsers
│   ├── vite.svg             # Vite logo
├── src/                     # Source code
│   ├── assets/              # Static assets
│   ├── components/          # Reusable components
│   │   ├── ui/              # UI components
│   │   │   ├── alert-dialog.jsx
│   │   │   ├── button.jsx
│   │   │   ├── card.jsx
│   │   │   ├── input.jsx
│   │   │   ├── label.jsx
│   │   │   ├── radio-group.jsx
│   │   │   ├── slider.jsx
│   │   │   └── RobotKinematicsPlayground.jsx
│   ├── lib/                 # Utility functions and hooks
│   ├── App.jsx              # Main application component
│   ├── global.css           # Global styles
│   ├── main.jsx             # Application entry point
├── .gitattributes           # Git configuration for file handling
├── .gitignore               # Git ignore file
├── README.md                # Documentation
├── components.json          # shadcn/ui component metadata
├── eslint.config.js         # ESLint configuration
├── index.html               # HTML template
```

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any changes.

## License

This project is licensed under the MIT License.


