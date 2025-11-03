#pragma once

#include "IBackupDatabaseInner.h"
#include "ComponentStub.h"

#define DECLARE_UNIQUE_BACKUPDATABASE_STUB() DECLARE_UNIQUE_COMPONENT_STUB(BackupDatabase)
#define IMPLEMENT_UNIQUE_BACKUPDATABASE_STUB() \
    IMPLEMENT_UNIQUE_COMPONENT_STUB(BackupDatabase, UNIQUE_BACKUPDATABASE_NAME)
