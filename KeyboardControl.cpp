#include "KeyboardControl.h"

KeyboardControl* KeyboardControl::transferPointer = nullptr;

KeyboardControl::KeyboardControl(): m_desiredSpeed({0.0, 0.0}), m_actualSpeed({0.0, 0.0}), m_iMicros({1500, 1500}),
                                    m_pidControl({PIDControl{500.0, 1850.0, 0.0, 0.04}, PIDControl{500.0, 1850.0, 0.0, 0.04}})
    {
		m_simInterface.Initialize(0.04, &KeyboardControl::transferFunction);
		KeyboardControl::transferPointer = this;
    }

void KeyboardControl::SpeedProtect(double &speed, double adjustspeed) {
	speed += adjustspeed;
	if (fabs(speed) > 0.5) {
		speed = std::max(-0.5, std::min(0.5, speed));
	}
}


void KeyboardControl::transferFunction() {
    KeyboardControl::transferPointer->Step();
}


void KeyboardControl::Step() {
    //read actualspeeds of robot(m/s)
    double* pInput = m_simInterface.GetInput();
    m_actualSpeed[0] = pInput[0]; //right
    m_actualSpeed[1] = pInput[1]; //link
	// transformation from speed(m/s) to signalLenghths(us)
    for (int i = 0; i < 2; ++i){ //two PIDcontrollers for two wheels
        m_pidControl[i].calculateU(m_desiredSpeed[i], m_actualSpeed[i]); // 0 right 1 left
        m_iMicros[i] = static_cast<int>(m_pidControl[i].getU() + 1500);  // Transfermation from double to int[us]
	    // Range [1000, 2000]us
        m_iMicros[i] = std::max(1000, std::min(2000, m_iMicros[i])); 
    }
    m_simInterface.SetOutputs(m_iMicros.data());  //pointer for array, first element
}


void KeyboardControl::Communicate() {
    sigprocmask(SIG_UNBLOCK, &m_simInterface.mask, nullptr);
    initscr();
    noecho();
    nodelay(stdscr, TRUE);

    char ch;
    printw("Keyboard control begins!\n");
	printw("Please press [w][a][s][d] to control Robot, \n[b] for stop, [q] for quit!");
    refresh();
	
    auto last_time = std::chrono::system_clock::now();

    while(ch != 'q'){
        ch = getch();
        if (ch != -1) { 
            clear();
            switch (ch) {
            case 'w' : {
                SpeedProtect(m_desiredSpeed[0], 0.01);
                SpeedProtect(m_desiredSpeed[1], 0.01);
                break;   //speed up
            }
            case 's' : {
                SpeedProtect(m_desiredSpeed[0], -0.01);
                SpeedProtect(m_desiredSpeed[1], -0.01);
                break;   //speed down
            }
            case 'd' : {
                SpeedProtect(m_desiredSpeed[0], -0.005);
                SpeedProtect(m_desiredSpeed[1], 0.005);
                break;   //turn left
            }
            case 'a' : {
                SpeedProtect(m_desiredSpeed[0], 0.005);
                SpeedProtect(m_desiredSpeed[1], -0.005);
                break;   //turn right
            }
            case 'b' : {
                m_desiredSpeed[0] = 0;
                m_desiredSpeed[1] = 0;
                break;   //stop
            }
            }
            clear();
            printw("Disered right speed: %.5f, Disered left speed: %.5f\n",
                    m_desiredSpeed[0], m_desiredSpeed[1]);
            printw("Actual right speed: %.5f, Actual left speed: %.5f\n",
                    m_actualSpeed[0], m_actualSpeed[1]);
            printw("Last input: %c\n",char(ch));
            refresh();   
        } 
        else {   //Refresh the screen when there is no input
            auto current_time = std::chrono::system_clock::now();
			if (current_time - last_time >= std::chrono::seconds(2)) {
			printw("Actual Right Speed: %.5f, Actual Left Speed: %.4f, Last input: %c\n",
					m_actualSpeed[0], m_actualSpeed[1], char(ch));
            refresh();
			last_time = current_time;
            }    
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));   
    }
    while(fabs(m_actualSpeed[0]) >= 0.0005 || fabs(m_actualSpeed[1]) >= 0.0005) {
        m_desiredSpeed[0] = 0; m_desiredSpeed[1] = 0;
        clear();
    	auto current_time = std::chrono::system_clock::now();  
        if (current_time - last_time >= std::chrono::seconds(1)) {
		    printw("Robot is still driving now!");
            printw("Disered right speed: %.5f, Disered left speed: %.5f\n",
                    m_desiredSpeed[0], m_desiredSpeed[1]);
            printw("Actual right speed: %.5f, Actual left speed: %.5f\n",
                    m_actualSpeed[0], m_actualSpeed[1]);
            printw("Last input: %c\n",char(ch));
				refresh();
				last_time = current_time;      
        }
    }
	printw("Robot stops now!");
	refresh();
	endwin();
    sigprocmask(SIG_BLOCK, &m_simInterface.mask, nullptr);
}









