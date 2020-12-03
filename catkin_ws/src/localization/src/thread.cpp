#include <csignal>
#include <cstdlib>
#include <iostream>

#include "../include/thread.h"

Thread::Thread()
    : m_pThread(nullptr)
    , m_threadName("")
{
}

Thread::~Thread()
{
    if (m_pThread)
    {
        delete m_pThread;
        m_pThread = nullptr;
    }
}

void Thread::start()
{
    sigset_t new_set;
    sigset_t old_set;

    /* Block signals in new thread... */
    sigfillset(&new_set);
    pthread_sigmask(SIG_SETMASK, &new_set, &old_set);

    m_pThread = new std::thread(&Thread::run, this);

    /* Restore signal handling in calling thread */
    pthread_sigmask(SIG_SETMASK, &old_set, NULL);
}

void Thread::shutdown()
{
    this->terminate();
}

void Thread::join()
{
    if (m_pThread)
        m_pThread->join();
}
