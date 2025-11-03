#ifndef __TOOLSDK_I_NON_DELEGATING_BASE_H__
#define __TOOLSDK_I_NON_DELEGATING_BASE_H__

interface INondelegatingBase {
    virtual bool nondelegatingQueryInterface(const char *name, void **pInterface) = 0;
    virtual void nondelegatingAddRef() = 0;
    virtual void nondelegatingReleaseRef() = 0;
};

#endif
