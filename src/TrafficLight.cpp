#include <iostream>
#include <random>
#include "TrafficLight.h"

/* Implementation of class "MessageQueue" */

template <typename T>
T MessageQueue<T>::receive()
{
    // FP.5a : The method receive should use std::unique_lock<std::mutex> and _condition.wait() 
    // to wait for and receive new messages and pull them from the queue using move semantics. 
    // The received object should then be returned by the receive function. 

    // perform queue modification under the lock
    std::unique_lock<std::mutex> uLock(_mutex);

    // pass unique lock to condition variable
    _condition.wait(uLock, [this] { return !_queue.empty(); });

    // remove last vector element from queue
    T msg = std::move(_queue.back());
    _queue.pop_back();

    // will not be copied due to rvo
    return msg;
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    // FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex> 
    // as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.


    // simulate some work
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // perform modification under the lock
    std::lock_guard<std::mutex> uLock(_mutex);

    // add to msg to queue 
    _queue.push_back(std::move(msg));

    // notify client after pushing new msg into vector 
    _condition.notify_one(); 
}

/* Implementation of class "TrafficLight" */

TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
}

void TrafficLight::waitForGreen()
{
    // FP.5b : add the implementation of the method waitForGreen, in which an infinite while-loop 
    // runs and repeatedly calls the receive function on the message queue. 
    // Once it receives TrafficLightPhase::green, the method returns.

    while(true)
    {
        TrafficLightPhase msg = _messages.receive();
        if (msg == TrafficLightPhase::green)
            return; 
    }
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    return _currentPhase;
}

void TrafficLight::simulate()
{
    // FP.2b : Finally, the private method „cycleThroughPhases“ should be started in a thread when the public method „simulate“ is called. To do this, use the thread queue in the base class. 

    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    // FP.2a : Implement the function with an infinite loop that measures the time between two loop cycles 
    // and toggles the current phase of the traffic light between red and green and sends an update method 
    // to the message queue using move semantics. The cycle duration should be a random value between 4 and 6 seconds. 
    // Also, the while-loop should use std::this_thread::sleep_for to wait 1ms between two cycles. 

    // random seed
    std::random_device rd;
    std::mt19937 eng(rd());

    // uniform distribution between 4000 and 6000
    std::uniform_int_distribution<int> distr(4000, 6000);

    // init cycle duration
    double cycleDuration = distr(eng); 

    // init stop watch 
    std::chrono::time_point<std::chrono::system_clock> lastUpdate;
    lastUpdate = std::chrono::system_clock::now();

    while(true)
    {
        // compute time difference to stop watch 
        long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count(); 

        // toggle the traffic light every 2 cycles and send message 
        if (timeSinceLastUpdate >= 2 * cycleDuration)
        {
            // sleep for 1ms every 2 cycles to reduce CPU usage
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            // toggle traffic light 
            switch(getCurrentPhase())
            {
                case TrafficLightPhase::red:
                    _currentPhase = TrafficLightPhase::green;
                    break;
                case TrafficLightPhase::green:
                    _currentPhase = TrafficLightPhase::red;
                    break;
            }

            // create instance and move it 
            TrafficLightPhase msg = getCurrentPhase();

            // push new phase into messages by calling send with move
            _messages.send(std::move(msg));

            // reset the stop watch
            lastUpdate = std::chrono::system_clock::now();
        }
    } // eof simulation loop
}

