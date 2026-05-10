# How does a Proportional-Integral-Differential (PID) control system contribute to precise movement in robotics
*The basics of PID controllers used in VEX robotics*

---
This is the GitHub page containing Cheyns Ruben's "How does a Proportional-Integral-Differential (PID) control system contribute to precise movement in robotics" thesis, poster and assorted code.

structure:
- **vscode**  
  vscode setup with vex extension to execute main.py
- **src**
  - **dynamicPlot.py**  
    A program to run on your pc, it interprets and plots graphs from main.py.
  - **main.py**  
    A bot-ready program to be run alongside dynamicPlot.py in order to tune a PID controller.
  - **noiseExample.py**  
    A program generating a function, applying random noise and plotting it for explanation purposes.
  - **PID.py**  
    A library containing PID controller classes to be used in other programs.
  - **plotter.py**  
    A program to plot CSV files generated on PID run, allowing for better tuning
  - **UI.py**  
    A library containing UI elements for the VEX brain
- **2025-2026_PID_Thesis_CheynsRuben.pdf**  
  The full thesis document
- **2025-2026_PID_Poster_CheynsRuben.pdf**  
  The poster presentation
- **LICENSE**  
  MIT License with academic attribution requirements
- **CITATION.cff**  
  Citation format for academic use
- **README**