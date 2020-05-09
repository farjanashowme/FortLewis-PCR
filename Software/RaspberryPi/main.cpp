// FLC-PCR
// Fort Lewis Collage
// James Ferguson

#include <stdio.h>
#include <iostream>
#include <string>
#include <ctime>
#include <sys/time.h>
#include <filesystem>

#include "ui.h"

// lunix serial
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

enum states {QUIT, MAIN_MENU, EDITOR_MENU, LOAD_MENU, SAVE_MENU, PREVIOUS};

// class for performing logic and rendering of the experiment editor where users can edit and run experiments
class ExperimentEditor {
  private:
    int serialPort_; // the serial port for communication to atmega328p
    bool atTemperature_; // is the system at the target temperature
    bool timerStarted_; // has the timer for the current step started
    timeval stepStartTime_; // the time that the current step got to temperature
    int touchTimeStart_ = 0; // time touch started

    enum states_ {IDLE_, RUNNING_, DONE_, ABORT_}; // system states
    states_ state_ = IDLE_; // current system state<F3>
    int experimentIndex_ = 0; // current temperature step position
 
    // UI element declarations
    UI::CycleArray cycleArray_;
    
    // keypad, start/stop, and new step elements
    UI::Padding buttonPadding_;
    UI::NumberKey keys_ [12];
    UI::Padding buttonCover_;
    UI::Image recycleBin_;
    UI::Button startStopButton_;
    UI::Button loadButton_;
    UI::Button saveButton_;
    UI::Text dragToAdd_;
    UI::TextBox* selectedTextBox_ = nullptr;
    UI::CycleStep* newStep_ = nullptr;
    UI::CycleStep* heldStep_ = nullptr;
    UI::Cycle* heldCycle_ = nullptr;
    
    // info bar and contained elements
    UI::Padding infoBarPadding_;
    UI::Image statusIndicator_;
    UI::Padding progressBar_Padding_;
    UI::Padding progressBar_;
    UI::Text targetTemperature_;
    UI::Text saveFileText_;

  public:
    ExperimentEditor ():
    cycleArray_(5, 5),
    buttonPadding_("img/padding/R_Grey_2.png", 5, 554, -10, 251, 500),
    keys_{UI::NumberKey(560,  78, 75, 75, '7', "7"),
         UI::NumberKey(640,  78, 75, 75, '8', "8"),
         UI::NumberKey(720,  78, 75, 75, '9', "9"),
         UI::NumberKey(560, 158, 75, 75, '4', "4"),
         UI::NumberKey(640, 158, 75, 75, '5', "5"),
         UI::NumberKey(720, 158, 75, 75, '6', "6"),
         UI::NumberKey(560, 238, 75, 75, '1', "1"),
         UI::NumberKey(640, 238, 75, 75, '2', "2"),
         UI::NumberKey(720, 238, 75, 75, '3', "3"),
         UI::NumberKey(560, 318, 75, 75, '0', "0"),
         UI::NumberKey(640, 318, 75, 75, '.', "."),
         UI::NumberKey(720, 318, 75, 75, '\b', "del")},
    buttonCover_("img/padding/R_Blue.png", 5, 554, -10, 251, 500),
    recycleBin_("img/Recycle.png", 616, 176),
    startStopButton_(665, 5, 130, 68, "Start"),
    loadButton_(560, 398, 115, 50, "Load"),
    saveButton_(680, 398, 115, 50, "Save"),
    dragToAdd_("fonts/consola.ttf", 16, 560, 5, "Drag to Add"),
    infoBarPadding_("img/padding/R_Grey_1.png", 5, -10, 453, 820, 36),
    statusIndicator_("img/Red_Light.png", 5, 459), 
    progressBar_Padding_("img/padding/S_Grey_2.png", 2, 26, 459, 69, 16),
    progressBar_("img/padding/S_Red.png", 2, 26, 459, 0, 16),
    targetTemperature_("fonts/consola.ttf", 16, 100, 459, "Idle"),
    saveFileText_("fonts/consola.ttf", 16, 795, 459, "", true) {
      openSerial();
    }

    // turn on and start sending temperature data to plc
    void startExperiment () {
      if (cycleArray_.size() == 0) {
        return;
      }
      if (selectedTextBox_ != nullptr) {
        selectedTextBox_->deselect();
        selectedTextBox_ = nullptr;
      }
      state_ = RUNNING_;
      experimentIndex_ = -1;
      writeSerial("on\n");
      statusIndicator_.setTexture("img/Green_Light.png");
      progressBar_.setTexture("img/padding/S_Green.png", 2);
      startStopButton_.setText("Abort");
      gettimeofday(&stepStartTime_, NULL);
      nextStep(); 
    }

    // check if an update to plc temperature is readdy
    void updateStep () {
      timeval currentTime;
      gettimeofday(&currentTime, NULL);
      double currentDuration = (currentTime.tv_sec - stepStartTime_.tv_sec) + (currentTime.tv_usec - stepStartTime_.tv_usec) * 1e-6;
      
      static int cnt;
      cnt = (cnt + 1) % 120;
      if (cnt == 59) {
        writeSerial("d\n");
      } else if (cnt == 119) {
        std::string serialString = readSerial(); 
        
        std::string sub = "none";
        for (unsigned int i = 0; i < serialString.length(); i++) {
          if (serialString[i] == ' ') {
            sub = serialString.substr(0, i);
            UI::CycleStep* step = cycleArray_.getStep(experimentIndex_);
            if (std::stof(sub) - std::stof(step->getTemperature()->getText()) < 5) {
              atTemperature_ = true;
            }
            break;
          }
        }  
      }

      if (atTemperature_ && !timerStarted_) {
        gettimeofday(&stepStartTime_, NULL);
        timerStarted_ = true;
      }
      
      if (timerStarted_ && currentDuration > std::stoi(cycleArray_.getStep(experimentIndex_)->getDuration()->getText())) {
        nextStep();
        gettimeofday(&stepStartTime_, NULL);
      }
    }

    // set plc to next temperature in experiment
    void nextStep () {
      experimentIndex_ += 1;
      
      progressBar_.setWH(69 * (float)experimentIndex_ / (float)cycleArray_.size(), 16);

      try {
        UI::CycleStep* step = cycleArray_.getStep(experimentIndex_);
        targetTemperature_.setText(step->getTemperature()->getText() + "\xB0" + "C");
        writeSerial("pt" + step->getTemperature()->getText() + "\n");
        atTemperature_ = false;
        timerStarted_ = false;
      } catch (...) {
        finishExperiment();
      }
    }

    // reached end of experiment shut plc down
    void finishExperiment () {
      writeSerial("off\n");
      targetTemperature_.setText("Finished");
      statusIndicator_.setTexture("img/Blue_Light.png");
      progressBar_.setTexture("img/padding/S_Blue.png", 2);
      state_ = DONE_;
      startStopButton_.setText("Reset");
    } 

    // pematurely end experiment turn plc off
    void abortExperiment () {
      writeSerial("off\n");
      targetTemperature_.setText("Aborted");
      statusIndicator_.setTexture("img/Red_Light.png");
      progressBar_.setTexture("img/padding/S_Red.png", 2);
      state_ = ABORT_;
      startStopButton_.setText("Reset");
    }

    // re enables experiment editiog
    void resetExperiment() {
      experimentIndex_ = 0;
      progressBar_.setWH(0, 16);
      targetTemperature_.setText("Idle");
      statusIndicator_.setTexture("img/Red_Light.png");
      progressBar_.setTexture("img/padding/S_Red.png", 2);
      state_ = IDLE_;
      startStopButton_.setText("Start");
    }
    
    // open /dev/ttyACM0 as linux serial with baud rate 115200
    void openSerial () {
      serialPort_ = open("/dev/ttyACM0", O_RDWR);

      if (serialPort_ < 0) {
        printf("Error %i opening serial port: %s\n", errno, strerror(errno));
      }

      struct termios tty;
      memset(&tty, 0, sizeof(tty));

      if (tcgetattr(serialPort_, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      }

      tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
      tty.c_cflag |= CREAD | CLOCAL | CS8;

      tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);

      tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

      tty.c_oflag &= ~(OPOST | ONLCR);

      tty.c_cc[VTIME] = 0;
      tty.c_cc[VMIN] = 0;

      cfsetispeed(&tty, B115200);
      cfsetospeed(&tty, B115200);

      if (tcsetattr(serialPort_, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      }
    }

    // close serial port
    void closeSerial () {
      close(serialPort_);
    }

    // read data from serial port
    std::string readSerial () {
      char readBuffer [256];
      memset(&readBuffer, '\0', sizeof(readBuffer));
      read(serialPort_, &readBuffer, 256);
      return std::string(readBuffer);
    }

    // write data to serial port
    void writeSerial (std::string message) {
      write(serialPort_, message.c_str(), sizeof(message.c_str()));
    }
    
    // performs editor input and logic returns state to go to next
    states logic () {
      SDL_Point touchLocation; // location of touch
      while (SDL_PollEvent(&UI::event) != 0) { // loop through all inputs
        if (UI::event.type == SDL_QUIT) { // if user hit window x button
          return QUIT; // quit
        } else if (UI::event.type == SDL_KEYDOWN) { // key presses
          switch (UI::event.key.keysym.sym) {
            case SDLK_ESCAPE: // pressed escape, quit
              return QUIT;
              break;
            case SDLK_s: // pressed s, take screenshot
              static int screenshotNumber;
              UI::takeScreenShot("screenshot" + std::to_string(screenshotNumber++) + ".png");
              break;
          }

        // if touch start
        } else if (UI::event.type == SDL_FINGERDOWN) {
          
          // get event location and start time
          touchLocation.x = UI::event.tfinger.x * SCREEN_WIDTH;
          touchLocation.y = UI::event.tfinger.y * SCREEN_HEIGHT;
          touchTimeStart_ = UI::event.tfinger.timestamp;

          // only use buttons if not holding anything
          if (heldStep_ == nullptr && heldCycle_ == nullptr) {
            
            // type numbers if a text box is selected and a putton is pressed
            if (selectedTextBox_ != nullptr) {
              for (int i = 0; i < 12; i++) {
                SDL_Rect keyRect = keys_[i].getRect();
                if (SDL_PointInRect(&touchLocation, &keyRect)) {
                  keys_[i].press(selectedTextBox_);
                }
              }
            }

            // check if start/abort/reset button is pressed
            SDL_Rect startStopButton_Rect = startStopButton_.getRect();
            if (SDL_PointInRect(&touchLocation, &startStopButton_Rect)) {
              if (state_ == RUNNING_) {
                abortExperiment(); 
              } else if (state_ == DONE_ || state_ == ABORT_) {
                resetExperiment();
              } else {
                startExperiment();
              }
            }

            // load button
            SDL_Rect loadButton_Rect = loadButton_.getRect();
            if (SDL_PointInRect(&touchLocation, &loadButton_Rect)) {
              return LOAD_MENU;
            }

            // save button
            SDL_Rect saveButton_Rect = saveButton_.getRect();
            if (SDL_PointInRect(&touchLocation, &saveButton_Rect)) {
              return SAVE_MENU;
            }
          }

        // touch move
        } else if (UI::event.type == SDL_FINGERMOTION) {
          
          // get event location and velocity
          touchLocation.x = UI::event.tfinger.x * SCREEN_WIDTH;
          touchLocation.y = UI::event.tfinger.y * SCREEN_HEIGHT;
          SDL_Point touchDelta;
          touchDelta.x = UI::event.tfinger.dx * SCREEN_WIDTH;
          touchDelta.y = UI::event.tfinger.dy * SCREEN_HEIGHT;
          
          // only allow motion if an experiment is not running
          if (state_ != RUNNING_) {

            // if touch speed is grater than 5
            SDL_Rect buttonPadding_Rect = buttonPadding_.getRect();
            if (abs(touchDelta.x) + abs(touchDelta.y) > 5) {
              
              // if touch is over button padding
              if (SDL_PointInRect(&touchLocation, &buttonPadding_Rect)) {
                SDL_Rect newStep_Rect = newStep_->getRect();

                // if touching new step pick it up and not holding anything
                if (SDL_PointInRect(&touchLocation, &newStep_Rect) && heldStep_ == nullptr && heldCycle_ == nullptr) {
                  heldStep_ = newStep_;
                }
              } else { // touch is not on the button padding
                // if not holding anything
                if (heldStep_ == nullptr && heldCycle_ == nullptr) {
                  
                  // look at all the cycles
                  for (unsigned int i = 0; i < cycleArray_.cycles_.size(); i++) {
                    // look at all the steps of this cycle
                    for (unsigned int j = 0; j < cycleArray_.cycles_[i]->steps_.size(); j++) {
                      
                      // if touchin this step pick it up
                      SDL_Rect stepRect = cycleArray_.cycles_[i]->steps_[j]->getRect();
                      if (SDL_PointInRect(&touchLocation, &stepRect)) {
                        heldStep_ = cycleArray_.cycles_[i]->removeStep(j);
                      }
                    }

                    // if touching this cycle pick it up
                    SDL_Rect cycleRect = cycleArray_.cycles_[i]->getRect();
                    if (SDL_PointInRect(&touchLocation, &cycleRect) && touchLocation.y < cycleRect.y + 36) {
                      heldCycle_ = cycleArray_.removeCycle(i);
                    }
                  }

                  // pan around cycle view
                  float cycleArray_PosX = touchDelta.x + cycleArray_.getPoint().x; 
                  if (cycleArray_PosX < -110 * (int)cycleArray_.cycles_.size() + 345) {
                    cycleArray_PosX = -110 * (int)cycleArray_.cycles_.size() + 345;
                  }
                  if (cycleArray_PosX > 5) {
                    cycleArray_PosX = 5;
                  }
                  cycleArray_.setXY(cycleArray_PosX, 5);
                }
              }
            }

            //move held cycle or step if any
            if (heldCycle_ != nullptr) {
              heldCycle_->setXY(touchLocation.x - 50, touchLocation.y - 23);
            }
            if (heldStep_ != nullptr) {
              heldStep_->setXY(touchLocation.x - 50, touchLocation.y - 23);
            }
          }

        // touch end
        } else if (UI::event.type == SDL_FINGERUP) {
          
          // record touch location
          touchLocation.x = UI::event.tfinger.x * SCREEN_WIDTH;
          touchLocation.y = UI::event.tfinger.y * SCREEN_HEIGHT;
          
          // if not holding anuthing and the system is not running
          if (heldStep_ == nullptr && heldCycle_ == nullptr && state_ != RUNNING_) {
            
            // if the touch event was under half a seccond
            if (UI::event.tfinger.timestamp - touchTimeStart_ <= 500) { // tap
              
              // look at all the cycles
              for (unsigned int i = 0; i < cycleArray_.cycles_.size(); i++) {
                
                // look at all the steps in this cycle
                for (unsigned int j = 0; j < cycleArray_.cycles_[i]->steps_.size(); j++) {
                  
                  // if taped on this step
                  SDL_Rect stepRect = cycleArray_.cycles_[i]->steps_[j]->getRect();
                  if (SDL_PointInRect(&touchLocation, &stepRect)) {
                    // if tapped on the duration text box, select it
                    if (touchLocation.y > stepRect.y + stepRect.h / 2) {
                      if (selectedTextBox_ != nullptr) { // deselect old text box
                        selectedTextBox_->formatNumber();
                        selectedTextBox_->deselect();
                      } 
                      // select new text box
                      selectedTextBox_ = cycleArray_.cycles_[i]->steps_[j]->getDuration();
                      selectedTextBox_->select();
                    // if tapped on the temperature text box, select it
                    } else {
                      if (selectedTextBox_ != nullptr) { // deselect old text box
                        selectedTextBox_->formatNumber();
                        selectedTextBox_->deselect();
                      }
                      // select new text box
                      selectedTextBox_ = cycleArray_.cycles_[i]->steps_[j]->getTemperature();
                      selectedTextBox_->select();
                    }
                  }
                }
                // if tapped on the number of cycles text box, select it
                SDL_Rect cycleRect = cycleArray_.cycles_[i]->getRect();
                if (SDL_PointInRect(&touchLocation, &cycleRect) && touchLocation.y < cycleRect.y + 36) {
                  if (selectedTextBox_ != nullptr) { // deselect old text box
                    selectedTextBox_->formatNumber();
                    selectedTextBox_->deselect();
                  }
                  // select new text box
                  selectedTextBox_ = cycleArray_.cycles_[i]->getNumberOfCycles();
                  selectedTextBox_->select();
                }
              }
            }
          }

          // if touch end is over button padding delete
          SDL_Rect buttonPadding_Rect = buttonPadding_.getRect();
          if (SDL_PointInRect(&touchLocation, &buttonPadding_Rect)) {
            // delete whatever, if any is held
            if (heldStep_ != nullptr) {
              delete heldStep_;
              heldStep_ = nullptr;
            } else if (heldCycle_ != nullptr) {
              delete heldCycle_;
              heldCycle_ = nullptr;
            }
          }

          // if holding a step and not over button padding
          if (heldStep_ != nullptr) {
            SDL_Point cycleArray_Pos = cycleArray_.getPoint();
            // look at all the cycles
            for (unsigned int i = 0; i < cycleArray_.cycles_.size(); i++) {
              if (touchLocation.x - cycleArray_Pos.x < (int)i * 110 + 110) {
                // look at all the steps in this cycle
                for (unsigned int j = 0; j < cycleArray_.cycles_[i]->steps_.size(); j++) {
                  // if touch is inside cycle, put step inside cycle
                  if (touchLocation.y - cycleArray_Pos.y < (int)j * 52 + 54) {
                    cycleArray_.cycles_[i]->addStep(j, heldStep_);
                    heldStep_ = nullptr;
                    break;
                  }
                }
                // touch is below cycle, add step to end of cycle 
                if (heldStep_ != nullptr) {
                  cycleArray_.cycles_[i]->addStep(cycleArray_.cycles_[i]->steps_.size(), heldStep_);
                  heldStep_ = nullptr;
                }
                break;
              }
            }
            // touch is not on any cycle, create new cycle at end and place step in it
            if (heldStep_ != nullptr) {
              cycleArray_.addCycle(cycleArray_.cycles_.size(), new UI::Cycle());
              cycleArray_.cycles_[cycleArray_.cycles_.size() - 1]->addStep(0, heldStep_);
              heldStep_ = nullptr;
            }
          }

          // if holding cycle and not over button padding
          if (heldCycle_ != nullptr) {
            SDL_Point cycleArray_Pos = cycleArray_.getPoint();
            // look at all the cycles
            for (unsigned int i = 0; i < cycleArray_.cycles_.size(); i++) {
              // if touch is inside cycleArray_, put cycle inside the array
              if (touchLocation.x - cycleArray_Pos.x < (int)i * 110 + 55) {
                cycleArray_.addCycle(i, heldCycle_);
                heldCycle_ = nullptr;
                break;
              }
            }
            // touch outside cycle array, put cycle at the end of cycle array
            if (heldCycle_ != nullptr) {
              cycleArray_.addCycle(cycleArray_.cycles_.size(), heldCycle_);
              heldCycle_ = nullptr;
            }
          }
        }
      }

      // if the new step was taken make a new one
      if (newStep_ == heldStep_ || newStep_ == nullptr) {
        newStep_ = new UI::CycleStep(560, 26);
      }

      // remove empty cycle from the cycle array
      cycleArray_.removeEmptyCycles();

      // update plc if an experiment is running
      if (state_ == RUNNING_) {
        updateStep();
      }
      return EDITOR_MENU;
    }

    // renders the UI for the cycle editor
    void render () {
      // begin render, clear screen
      SDL_SetRenderDrawColor(UI::renderer, 74, 84, 98, 255);
      SDL_RenderClear(UI::renderer);
      
      cycleArray_.render();
      buttonPadding_.render();
      startStopButton_.render();
      loadButton_.render();
      saveButton_.render();
      for (int i = 0; i < 12; i++) {
        keys_[i].render();
      }
      
      dragToAdd_.render();
      newStep_->render();

      if (heldStep_ != nullptr || heldCycle_ != nullptr) {
        buttonCover_.render();
        recycleBin_.render();
      }

      infoBarPadding_.render();
      statusIndicator_.render();
      progressBar_Padding_.render();
      saveFileText_.render();

      int progressBarSize = progressBar_.getRect().w;
      if (progressBarSize >= 3) {
        progressBar_.render();
      }
      targetTemperature_.render();
 
      if (heldStep_ != nullptr) {
        heldStep_->render();
      }
      if (heldCycle_ != nullptr) {
        heldCycle_->render();
      }
      // present render
      SDL_RenderPresent(UI::renderer);
    }

    // save experiment in cycle array
    void save (std::string path) {
      saveFileText_.setText(path);
      cycleArray_.save(path);
    }

    // load experiment into cycle array
    void load (std::string path) {
      saveFileText_.setText(path);
      cycleArray_.load(path);
    }

    ~ExperimentEditor () {
      if (selectedTextBox_ != nullptr) {
        delete selectedTextBox_;
      }
      delete newStep_;
      if (selectedTextBox_ != nullptr) {
        delete heldStep_;
      }
      if (selectedTextBox_ != nullptr) {
        delete heldCycle_;
      }
      closeSerial();
    }
};

class MainMenu {
  private:
    ExperimentEditor* editor_; // the editor to enteract with
    
    // UI elements
    UI::Text mainText_;
    UI::Button newButton_;
    UI::Button loadButton_;

  public:
    MainMenu (ExperimentEditor* editor) :
    mainText_("fonts/consola.ttf", 100, 10, 10, "FLC-PCR Main Menu"),
    newButton_(10, 120, 780, 100, "New Experiment"),
    loadButton_(10, 230, 780, 100, "Load Experiment") {
      editor_ = editor;
    }

    // performs main menu input and logic returns state to go to next
    states logic () {
      while (SDL_PollEvent(&UI::event) != 0) { // loop through all inputs
        if (UI::event.type == SDL_QUIT) { // if user hit window x button
          return QUIT; // quit
        } else if (UI::event.type == SDL_KEYDOWN) { // key presses
          switch (UI::event.key.keysym.sym) {
            case SDLK_ESCAPE: // pressed escape, quit
              return QUIT;
              break;
            case SDLK_s: // pressed s, take screenshot
              static int screenshotNumber;
              UI::takeScreenShot("screenshot" + std::to_string(screenshotNumber++) + ".png");
              break;
          }
        } else if (UI::event.type == SDL_FINGERDOWN) {
          // button presses
          SDL_Point touchLocation;
          touchLocation.x = UI::event.tfinger.x * SCREEN_WIDTH;
          touchLocation.y = UI::event.tfinger.y * SCREEN_HEIGHT;

          // new experiment button pressed
          SDL_Rect newButtonRect = newButton_.getRect();
          if (SDL_PointInRect(&touchLocation, &newButtonRect)) {
            editor_->load("/home/pi/Untitled.exp");
            return EDITOR_MENU;
          }

          // load experiment pressed
          SDL_Rect loadButtonRect = loadButton_.getRect();
          if (SDL_PointInRect(&touchLocation, &loadButtonRect)) {
            return LOAD_MENU;
          }
        }
      }
      return MAIN_MENU;
    }

    void render () {
      // begin render, clear screen
      SDL_SetRenderDrawColor(UI::renderer, 139, 147, 175, 255);
      SDL_RenderClear(UI::renderer);
      
      mainText_.render();
      newButton_.render();
      loadButton_.render();

      // present render
      SDL_RenderPresent(UI::renderer);
    }
};

class LoadSaveMenu {
  private:
    ExperimentEditor* editor_; // the editor to enteract with
    UI::Padding pathSelection_; // padding behind the selected path
    int selectedPathIndex_ = -1; // id of the selected path, -1 for no selection
    std::vector<UI::Text> experimentPaths_; // file paths
    std::string savePath_ = "/home/pi/experiments/"; // folder to look in for files
    float textScroll_ = 55; // vertical position of experiment paths
    int touchTimeStart_ = 0; // for keeping track of taps
    
    // UI elements
    UI::Padding buttonPadding_;
    UI::Button newButton_;
    UI::Button loadButton_;
    UI::Button saveButton_;
    UI::Button backButton_;
    UI::Button deleteButton_;

    // save new file popup UI elements
    bool newSaveFile_ = false; // is the save new file popup open
    UI::Padding newSaveCover_;
    UI::Text fileText_;
    UI::TextBox newSavePath_;
    const int KEY_SIZE_ = 71;
    const static int NUM_OF_KEYS_ = 45;
    UI::Key keys_ [NUM_OF_KEYS_];
    UI::Button doneButton_;
    UI::Button cancelButton_;

  public:
    LoadSaveMenu (ExperimentEditor* editor) :
    pathSelection_("img/padding/S_Blue.png", 2, -2, 8, 804, 20),
    buttonPadding_("img/padding/R_Grey_2.png", 5, -5, -5, 810, 51),
    newButton_(5, 5, 100, 35, "New"),
    loadButton_(110, 5, 100, 35, "Load"),
    saveButton_(215, 5, 100, 35, "Save"),
    backButton_(320, 5, 100, 35, "Cancel"),
    deleteButton_(695, 5, 100, 35, "Delete"),
    newSaveCover_("img/padding/R_Grey_2.png", 5, 10, 10, 780, 460),
    fileText_("fonts/consola.ttf", 16, 15, 15, "Filename :"),
    newSavePath_(110, 15, 670, 21),
    keys_{UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, '1', "1"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, '2', "2"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, '3', "3"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, '4', "4"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, '5', "5"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, '6', "6"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, '7', "7"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, '8', "8"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, '9', "9"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, '0', "0"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'q', "q"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'w', "w"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'e', "e"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'r', "r"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 't', "t"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'y', "y"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'u', "u"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'i', "i"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'o', "o"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'p', "p"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'a', "a"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 's', "s"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'd', "d"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'f', "f"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'g', "g"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'h', "h"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'j', "j"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'k', "k"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'l', "l"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, ';', ";"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'z', "z"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'x', "x"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'c', "c"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'v', "v"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'b', "b"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'n', "n"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, 'm', "m"),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, ',', ","),
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, '.', "."), 
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, ' ', ""), 
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, '-', "-"), 
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, '+', "+"), 
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, '=', "="), 
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, '_', "_"), 
         UI::Key(0, 0, KEY_SIZE_, KEY_SIZE_, '\b', "de")},
    doneButton_(15, 430, 100, 35, "Done"),
    cancelButton_(120, 430, 100, 35, "Cancel") {
      updatePaths();
      editor_ = editor;
      // position keys_
      for (int i = 0; i < NUM_OF_KEYS_; i++) {
        keys_[i].setXY(15 + (KEY_SIZE_ + 5) * (i % 10), 41 + (KEY_SIZE_ + 5) * (i / 10));
      }
    }
    
    // checks for new files and updates the list of experiment files
    void updatePaths () {
      experimentPaths_.clear();
      // iterate through all files including subdirectories
      for (const auto & entry : std::filesystem::recursive_directory_iterator(savePath_)) {
        // only add files with a .exp extention
        if (entry.path().string().substr(entry.path().string().size() - 4, 4) == ".exp") {
          UI::Text entryText("fonts/consola.ttf", 16, 0, 0, entry.path().string(), false);
          experimentPaths_.push_back(entryText);
        }
      }
      movePaths();
    }

    // position experiment paths
    void movePaths () {
      for (unsigned int i = 0; i < experimentPaths_.size(); i++) {
        experimentPaths_[i].setXY(15, i * 21 + textScroll_);
      }
      moveSelection();
    }

    // position experiment path selection
    void moveSelection () { 
      pathSelection_.setXY(-2, selectedPathIndex_ * 21 + textScroll_ - 2);
    }

    // performs load and save menu input and logic returns state to go to next
    states logic (states state) {
      while (SDL_PollEvent(&UI::event) != 0) { // loop through all inputs
        if (UI::event.type == SDL_QUIT) { // if user hit window x button
          return QUIT; // quit
        } else if (UI::event.type == SDL_KEYDOWN) { // key presses
          switch (UI::event.key.keysym.sym) {
            case SDLK_ESCAPE: // pressed escape, quit
              return QUIT;
              break;
            case SDLK_s: // pressed s, take screenshot
              static int screenshotNumber;
              UI::takeScreenShot("screenshot" + std::to_string(screenshotNumber++) + ".png");
              break;
          }
        } else if (UI::event.type == SDL_FINGERDOWN) {
          // button presses
          SDL_Point touchLocation;
          touchLocation.x = UI::event.tfinger.x * SCREEN_WIDTH;
          touchLocation.y = UI::event.tfinger.y * SCREEN_HEIGHT;
          if (!newSaveFile_) { // disable if popup is open
            touchTimeStart_ = UI::event.tfinger.timestamp;
            // new button
            SDL_Rect newButtonRect = newButton_.getRect();
            if (SDL_PointInRect(&touchLocation, &newButtonRect)) {
              editor_->load("/home/pi/Untitled.exp");
              selectedPathIndex_ = -1;
              textScroll_ = 55;
              movePaths();
              return EDITOR_MENU;
            }
            // load button
            SDL_Rect loadButtonRect = loadButton_.getRect();
            if (SDL_PointInRect(&touchLocation, &loadButtonRect)) {
              if (selectedPathIndex_ != -1) { 
                editor_->load(experimentPaths_[selectedPathIndex_].getText());
                selectedPathIndex_ = -1;
                textScroll_ = 55;
                movePaths();
                return EDITOR_MENU;
              }
            }
            // save button
            SDL_Rect saveButtonRect = saveButton_.getRect();
            if (SDL_PointInRect(&touchLocation, &saveButtonRect)) {
              if (selectedPathIndex_ != -1) { 
                editor_->save(experimentPaths_[selectedPathIndex_].getText());
              } else {
                newSaveFile_ = true;
              }
            }
            // delete button
            SDL_Rect deleteButtonRect = deleteButton_.getRect();
            if (SDL_PointInRect(&touchLocation, &deleteButtonRect)) {
              if (selectedPathIndex_ != -1) {
                remove(experimentPaths_[selectedPathIndex_].getText().c_str());
                selectedPathIndex_ = -1;
                updatePaths();
              } 
            }
            // back button
            SDL_Rect backButtonRect = backButton_.getRect();
            if (SDL_PointInRect(&touchLocation, &backButtonRect)) {
              selectedPathIndex_ = -1;
              textScroll_ = 55;
              movePaths();
              return PREVIOUS;
            }
          } else { // popup is open
            // key presses
            for (int i = 0; i < NUM_OF_KEYS_; i++) {
              SDL_Rect keyRect = keys_[i].getRect();
              if (SDL_PointInRect(&touchLocation, &keyRect)) {
                keys_[i].press(&newSavePath_);
              } 
            }
            // done button
            SDL_Rect doneRect = doneButton_.getRect();
            if (SDL_PointInRect(&touchLocation, &doneRect)) {  
              editor_->save(savePath_ + newSavePath_.getText() + ".exp"); 
              newSaveFile_ = false;
              updatePaths(); 
            }
            // cancel button
            SDL_Rect cancelRect = cancelButton_.getRect();
            if (SDL_PointInRect(&touchLocation, &cancelRect)) {  
              newSaveFile_ = false;
              updatePaths();
            }
          }
        } else if (UI::event.type == SDL_FINGERMOTION) {
          if (!newSaveFile_) {  // disable if popup is open
            // scroll through files
            if (abs(UI::event.tfinger.dy * SCREEN_HEIGHT) > 3) {
              textScroll_ += UI::event.tfinger.dy * SCREEN_HEIGHT;
              if (textScroll_ < 55 + 21 - (int)experimentPaths_.size() * 21) {
                textScroll_ = 55 + 21 - (int)experimentPaths_.size() * 21;
              }
              if (textScroll_ > 55) {
                textScroll_ = 55;
              }
              movePaths();
            }
          }
        } else if (UI::event.type == SDL_FINGERUP) {
         if (!newSaveFile_) { // disable if popup is open
            SDL_Point touchLocation;
            touchLocation.x = UI::event.tfinger.x * SCREEN_WIDTH;
            touchLocation.y = UI::event.tfinger.y * SCREEN_HEIGHT;
            // select path
            if (UI::event.tfinger.timestamp - touchTimeStart_ <= 200 && touchLocation.y > 50) {
              selectedPathIndex_ = int(touchLocation.y - textScroll_) / 21;
              if (selectedPathIndex_ >= (int)experimentPaths_.size() || selectedPathIndex_ < 0) {
                selectedPathIndex_ = -1;
              }
              moveSelection();
            }
          } 
        }
      }
      return state;
    }

    void render () {
      // begin render, clear screen
      SDL_SetRenderDrawColor(UI::renderer, 109, 117, 141, 255);
      SDL_RenderClear(UI::renderer);

      if (selectedPathIndex_ != -1) {
        pathSelection_.render();
      }

      for (unsigned int i = 0; i < experimentPaths_.size(); i++) {
        experimentPaths_[i].render();
      }

      buttonPadding_.render();
      newButton_.render();
      loadButton_.render();
      saveButton_.render();
      backButton_.render();
      deleteButton_.render();

      // draw new save file popup
      if (newSaveFile_) {
        newSaveCover_.render();
        fileText_.render();
        newSavePath_.render();
        for (int i = 0; i < NUM_OF_KEYS_; i++) {
          keys_[i].render();
        }
        doneButton_.render();
        cancelButton_.render();
      }

      // present render
      SDL_RenderPresent(UI::renderer);
    }
};

int main(int argc, char* args[]) {  
  // initialize UI
  if (UI::init()) {
    return 1;
  }
 
  states state = MAIN_MENU; // the curent state
  static states previousState = MAIN_MENU; // the previous state
  static states stateChangeCheck = MAIN_MENU; // for identifying a state change

  // create experiment editor
  ExperimentEditor editor;
  editor.logic(); // add to constructor to fix 
  
  // create menus
  MainMenu mainMenu(&editor);
  LoadSaveMenu loadSaveMenu(&editor);

  // program loop
  bool run = true;
  while (run) {
    // render and perform logic for the current state
    switch (state) {
      case QUIT:
        run = false;
        break;
      case MAIN_MENU:
        mainMenu.render();
        state = mainMenu.logic();
        break;
      case EDITOR_MENU:
        editor.render();
        state = editor.logic();
        break;
      case LOAD_MENU:
        loadSaveMenu.render();
        state = loadSaveMenu.logic(LOAD_MENU);
        break;
      case SAVE_MENU:
        loadSaveMenu.render();
        state = loadSaveMenu.logic(SAVE_MENU);
        break;
      case PREVIOUS:
        state = previousState;
        break;
      default:
        std::cout << "ERROR: Unknown state " << state << std::endl;
        run = false;
        break;
    }
    // keep trach of the last state
    if (state != stateChangeCheck && state != PREVIOUS) {
      previousState = stateChangeCheck;
      stateChangeCheck = state;
    }
    // check for errors dump to console
    if (SDL_GetError()[0] != '\0') {
      std::cout << "SDL ERROR: " << SDL_GetError() << std::endl;
      SDL_ClearError();
    }
  }
  
  // clean up
  SDL_DestroyWindow(UI::window);
  SDL_Quit();

  return 0;
}
