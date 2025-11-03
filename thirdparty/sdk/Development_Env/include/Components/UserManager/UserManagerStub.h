#pragma once

#include "IUserManagerInner.h"
#include "AuthorizedItem.h"
#include "ComponentStub.h"

#define DECLARE_UNIQUE_USERMANAGER_STUB() DECLARE_UNIQUE_COMPONENT_STUB(UserManager)
#define IMPLEMENT_UNIQUE_USERMANAGER_STUB() IMPLEMENT_UNIQUE_COMPONENT_STUB(UserManager, UNIQUE_USERMANAGER_NAME)
