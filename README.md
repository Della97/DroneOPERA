# drone-pp

## Getting Started

### Prerequisites

ns3 library (installed under /home/username/tarballs) and ns3-netvisualyzer-module (installed under src/contrib as tutorial on repo of the module).

### Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/Della97/drone-pp.git
   cd drone-pp
   
2. Build the library:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   
3. Run the code:
   ```bash
   make run_mpi

4. Run the simulation in NetSimulyzer:
   - Load the .xml file generated in src/build inside NetSimulyzer
   
