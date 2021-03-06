/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_LAZY_CALLER_H
#define CNOID_BASE_LAZY_CALLER_H

#include <boost/function.hpp>
#include "exportdecl.h"

namespace cnoid {
    
class LazyCallerImpl;


/**
   \note This is not thread safe
   \todo Make this thread safe so that the function can be called non-main threads
*/
class CNOID_EXPORT LazyCaller
{
    friend class LazyCallerImpl;
        
    bool isPending_;
    bool isNextPending;
    LazyCallerImpl* impl;
        
public:
    enum { PRIORITY_HIGH = 0, PRIORITY_NORMAL, PRIORITY_LOW };
        
    LazyCaller();
    LazyCaller(const boost::function<void(void)>& function, int priority = PRIORITY_HIGH);
    LazyCaller(const LazyCaller& org);
    virtual ~LazyCaller();

    void setFunction(const boost::function<void(void)>& function);
    void setPriority(int priority);

    /**
       The function is called once even if the lazy call is requested many times
       before the function is actually called.
       If the conservative mode is on, the function is called one before the function is called and finished.
    */
    void setConservative(bool on);

    bool isPending() const { return isPending_; }

    void flush();

    typedef void result_type;

    /**
       Multiple requests before the actual function call is summarized into a single call
    */
    void operator()() {
        if(!isPending_){
            isPending_ = true;
            postCallEvent();
        }
    }

    void cancel();

private:
    void postCallEvent();
};

//! deprecated
enum { IDLE_PRIORITY_HIGH = LazyCaller::PRIORITY_HIGH,
       IDLE_PRIORITY_NORMAL = LazyCaller::PRIORITY_NORMAL,
       IDLE_PRIORITY_LOW = LazyCaller::PRIORITY_LOW };

CNOID_EXPORT void callLater(const boost::function<void()>& function, int priority = LazyCaller::PRIORITY_NORMAL);
CNOID_EXPORT bool callSynchronously(const boost::function<void()>& function, int priority = LazyCaller::PRIORITY_NORMAL);

CNOID_EXPORT bool isRunningInMainThread();

}
        
#endif
