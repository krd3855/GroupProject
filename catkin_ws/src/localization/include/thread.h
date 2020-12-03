#ifndef THREAD_H
#define THREAD_H

#include <string>
#include <thread>

class Thread
{
public:
    Thread();
    virtual ~Thread();

    /**
     * Start thread loop
     */
    virtual void start();

    /**
     * Thread loop
     */
    virtual void run() = 0;

    /**
     * Terminate thread loop
     */
    virtual void terminate() = 0;

    /**
     * Shutdown thread loop
     */
    virtual void shutdown();

    /**
     * Calls join on thread, if started.
     */
    void join();

protected:
    std::string m_threadName; /*!< Thread name */
    std::thread* m_pThread;   /*!< Pointer to thread object */

}; /* class Thread */

#endif /* THREAD_H */