#pragma once

#include "ThirdPartyDef.h"
#include <thread>
#include <string>
#ifdef _DEBUG
#undef _DEBUG
#include "Python.h"
#define _DEBUG
#else
#include "Python.h"
#endif


struct _object;
typedef _object PyObject;
//仅对于业务逻辑封装
class THIRDPARTY_CLASS CallPythonWrapper {
public:
    CallPythonWrapper();
    ~CallPythonWrapper();

    bool bindMessage(const std::string& fileName, const std::string& filePath);
    std::string getMessageInfo();
    int calculateSize();

private:
    PyObject* m_pModule;
    PyObject* m_pClass;
    PyObject* m_instance;
};

class PythonThreadLocker {
    PyGILState_STATE state;

public:
    PythonThreadLocker() : state(PyGILState_Ensure()) {}
    ~PythonThreadLocker() { PyGILState_Release(state); }
};

class PythonContext {
public:
    PythonContext(bool thread = false) : m_save(nullptr), m_thread(thread) {
        Py_Initialize();
        if (!Py_IsInitialized()) {
            // std::cerr << "python init failed" << std::endl;
        }

        if (m_thread) {
#if !((PY_MAJOR_VERSION == 3 && PY_MINOR_VERSION >= 7) || (PY_MAJOR_VERSION > 3))
            PyEval_InitThreads();
            int nInit = PyEval_ThreadsInitialized();  //检测线程支持是否开启成功
            if (nInit != 0) {
                m_save =
                    PyEval_SaveThread();  //因为调用PyEval_InitThreads成功后，当前线程就拥有了GIL，释放当前线程的GIL，
            }
#endif
            // printf("[CallPython]start state:%d\n", PyGILState_Check());
        }
    }

    ~PythonContext() {
        if (m_thread) {
#if !((PY_MAJOR_VERSION == 3 && PY_MINOR_VERSION >= 7) || (PY_MAJOR_VERSION > 3))
            PyEval_RestoreThread(m_save);
#endif
            // printf("[CallPython]end state:%d\n", PyGILState_Check());
        }

        Py_FinalizeEx();
    }

private:
    PyThreadState* m_save;
    bool m_thread;
};