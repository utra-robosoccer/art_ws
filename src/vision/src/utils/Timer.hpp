#ifndef TIMERS_HPP
#define TIMERS_HPP

#include <string>
#include <ctime>


using namespace std;
class Timer{
	public:
		void start(){
			mStart = std::clock();
		};


		void printTime(string str){
    		double timeToProcessFrame = (std::clock() - mStart)/(double) CLOCKS_PER_SEC;
    
    		cout << str << "" << timeToProcessFrame << endl; 
    
		};

		Timer(){
    		mStart = std::clock();
		};
	private:

		std::clock_t mStart;
};


#endif // TIMERS_HPP