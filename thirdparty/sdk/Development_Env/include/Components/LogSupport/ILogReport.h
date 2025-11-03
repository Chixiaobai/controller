#pragma once

#include "ILog.h"

/**
 * @interface ILogReport
 * @brief Interface for receiving and processing log reports.
 *
 * This interface defines a callback method that is invoked when a log report is available.
 */
interface ILogReport : public IBase {
    /**
     * @brief Called when a log report is generated.
     *
     * @param pLog Pointer to the ILog instance containing the log data.
     * @return true if the report was handled successfully; false otherwise.
     */
    virtual bool onReport(ILog * pLog) = 0;
};

#ifndef LogSupport_EXPORTS
DECLARE_STUB_INTERFACE(ILogReportStub, ILogReport)
#endif

ENABLE_RECOGNIZE_MACRO(ILogReport)
