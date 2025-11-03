#pragma once

#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C extern
#endif

#if defined _WIN32
#ifdef EthercatDriver_EXPORTS
#define ETHERCATDRIVER_API EXTERN_C __declspec(dllexport)
#define ETHERCATDRIVER_CLASS __declspec(dllexport)
#else
#define ETHERCATDRIVER_API EXTERN_C __declspec(dllimport)
#define ETHERCATDRIVER_CLASS __declspec(dllimport)
#endif
#else
#if __GNUC__ >= 4
#define ETHERCATDRIVER_API EXTERN_C __attribute__((visibility("default")))
#define ETHERCATDRIVER_CLASS __attribute__((visibility("default")))
#else
#define ETHERCATDRIVER_API EXTERN_C
#define ETHERCATDRIVER_CLASS
#endif
#endif

#include "EthercatDef.h"

#define MIN_ETHERCAT_PERIOD_NS 100000
#define MAX_ETHERCAT_PERIOD_NS 4000000000

#ifndef EthercatDriver_EXPORTS

enum ETHERCAT_NORMAL_ERROR {
    ETHERCAT_NO_ERROR = 0,
    ETHERCAT_WC_UNMATCHED_ERROR = 1,
    ETHERCAT_MASTER_LINKDOWN_ERROR = 2,
    ETHERCAT_SLAVE_ERROR = 3,
    ETHERCAT_SLAVE_INCREASE = 4,
    ETHERCAT_SLAVE_DECREASE = 5,
};
#endif

typedef void (*ETHERCAT_ERROR_CALLBACK)(const unsigned int, const unsigned int, const int, void *);

/**
 * @brief Initializes an EtherCAT master with basic configuration.
 *
 * @param uintMasterIndex Index of the master to initialize.
 * @param uintExchangePeriod Communication cycle period in nanoseconds.
 * @param bSlaveAsDcMaster If true, configures the slave as a DC master.
 * @return true if initialization was successful; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_initialize(unsigned int uintMasterIndex, unsigned int uintExchangePeriod,
                                              bool bSlaveAsDcMaster);

/**
 * @brief Initializes an EtherCAT master with extended configuration options.
 *
 * @param uintMasterIndex Index of the master to initialize.
 * @param uintExchangePeriod Communication cycle period in nanoseconds.
 * @param bSyncEnabled If true, enables synchronization functionality.
 * @param bSlaveAsDcMaster If true, configures the slave as a DC master.
 * @return true if initialization was successful; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_initialize_ex(unsigned int uintMasterIndex, unsigned int uintExchangePeriod,
                                                 bool bSyncEnabled, bool bSlaveAsDcMaster);

/**
 * @brief Uninitializes the specified EtherCAT master.
 *
 * @param uintMasterIndex Index of the master to uninitialize.
 * @return true if uninitialization was successful; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_uninitialize(unsigned int uintMasterIndex);

/**
 * @brief Registers an error callback function for the specified master.
 *
 * This callback will be invoked when an EtherCAT error occurs.
 *
 * @param uintMasterIndex Index of the master.
 * @param callback Pointer to the error callback function.
 * @param param User-defined parameter passed to the callback.
 * @return true if the callback was registered successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_set_error_callback(unsigned int uintMasterIndex, ETHERCAT_ERROR_CALLBACK callback,
                                                      void *param);

/**
 * @brief Retrieves the current communication period of the specified master.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintExchangePeriod Output parameter that receives the period value in nanoseconds.
 * @return true if the period was retrieved successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_get_period(unsigned int uintMasterIndex, unsigned int *uintExchangePeriod);

/**
 * @brief Sets the exchange mode for the specified master.
 *
 * > **Warning:** Enabling `bReceiveInCircle` will block communication. Use with caution.
 *
 * @param uintMasterIndex Index of the master.
 * @param bReceiveInCircle If true, sets the master to run in circular receive mode.
 * @return true if the mode was set successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_set_exchange_mode(unsigned int uintMasterIndex, bool bReceiveInCircle);

/**
 * @brief Gets the system time base offset for distributed clocks (DC) synchronization.
 *
 * @param uintMasterIndex Index of the master.
 * @param intSystemBaseOffsetNs Output parameter that receives the offset in nanoseconds.
 * @return true if the offset was retrieved successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_get_system_base_offset(unsigned int uintMasterIndex, int64_t *intSystemBaseOffsetNs);

/**
 * @brief Appends an ENI (EtherCAT Network Information) file to the specified master.
 *
 * @param uintMasterIndex Index of the master.
 * @param strENIFilePath Path to the ENI file to append.
 */
ETHERCATDRIVER_API void igh_master_append_eni(unsigned int uintMasterIndex, const char *strENIFilePath);

/**
 * @brief Scans the network and detects connected slaves.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSlaveCount Output parameter that receives the number of detected slaves.
 * @return true if scanning was successful; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_scan(unsigned int uintMasterIndex, unsigned int *uintSlaveCount);

/**
 * @brief Retrieves the product code of a specific slave device.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSlaveIndex Zero-based index of the target slave.
 * @param uintProductCode Output parameter that receives the product code.
 * @return true if the product code was retrieved successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_get_slave_product_code(unsigned int uintMasterIndex, unsigned int uintSlaveIndex,
                                                          unsigned int *uintProductCode);

/**
 * @brief Retrieves the revision code of a specific slave device.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSlaveIndex Zero-based index of the target slave.
 * @param uintRevisionCode Output parameter that receives the revision code.
 * @return true if the revision code was retrieved successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_get_slave_revision_code(unsigned int uintMasterIndex, unsigned int uintSlaveIndex,
                                                           unsigned int *uintRevisionCode);

/**
 * @brief Configures a specific slave device.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSlaveIndex Zero-based index of the target slave.
 * @return true if configuration succeeded; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_config_slave(unsigned int uintMasterIndex, unsigned int uintSlaveIndex);

/**
 * @brief Configures all connected slave devices.
 *
 * @param uintMasterIndex Index of the master.
 * @return true if all slaves were configured successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_config_all_slave(unsigned int uintMasterIndex);

/**
 * @brief Activates the EtherCAT master and starts communication.
 *
 * @param uintMasterIndex Index of the master.
 * @return true if activation was successful; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_active(unsigned int uintMasterIndex);

/**
 * @brief Starts or continues EtherCAT data exchange.
 *
 * @param uintMasterIndex Index of the master.
 * @param callback Optional callback function to execute during each communication cycle.
 * @param lpParam User-defined parameter passed to the callback.
 * @return true if exchange started/continued successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_exchange(unsigned int uintMasterIndex, void (*callback)(void *), void *lpParam);

/**
 * @brief Retrieves the current state of the EtherCAT master.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintState Output parameter that receives the master state.
 * @return true if the state was retrieved successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_get_state(unsigned int uintMasterIndex, unsigned int *uintState);

/**
 * @brief Creates PDO (Process Data Object) information for reading/writing I/O data.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSlaveIndex Zero-based index of the target slave.
 * @param strSignalName Name of the signal associated with this PDO.
 * @param bUsedForRead If true, creates info for input PDO; otherwise for output.
 * @param uintPDOHandle Output parameter that receives the PDO handle.
 * @return true if PDO info was created successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_create_pdo_info(unsigned int uintMasterIndex, unsigned int uintSlaveIndex,
                                                   const char *strSignalName, bool bUsedForRead,
                                                   unsigned int *uintPDOHandle);

/**
 * @brief Destroys previously created PDO information.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintPDOHandle Handle of the PDO info to destroy.
 * @param bUsedForRead Specifies whether the PDO was used for reading.
 */
ETHERCATDRIVER_API void igh_master_destroy_pdo_info(unsigned int uintMasterIndex, unsigned int uintPDOHandle,
                                                    bool bUsedForRead);

/**
 * @brief Receives data from a PDO.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintPDOHandle Handle of the PDO to read from.
 * @param dblValue Output buffer that receives the double-precision floating point value.
 * @return true if data was received successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_pdo_recv(unsigned int uintMasterIndex, const unsigned int uintPDOHandle,
                                            double *dblValue);

/**
 * @brief Sends data to a PDO.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintPDOHandle Handle of the PDO to write to.
 * @param dblValue The double-precision floating point value to send.
 * @return true if data was sent successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_pdo_send(unsigned int uintMasterIndex, const unsigned int uintPDOHandle,
                                            const double dblValue);

/**
 * @brief Creates a raw SDO (Service Data Object) request for low-level access.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSlaveIndex Zero-based index of the target slave.
 * @param uintIndex Index of the object dictionary entry.
 * @param byteSubIndex Sub-index of the object dictionary entry.
 * @param uintSize Size of the data to transfer.
 * @param intTimeout Operation timeout in milliseconds.
 * @param bUsedForRead If true, prepares for reading; otherwise for writing.
 * @param uintSDOHandle Output parameter that receives the SDO handle.
 * @return true if the SDO request was created successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_create_raw_sdo_request(unsigned int uintMasterIndex, unsigned int uintSlaveIndex,
                                                          unsigned short uintIndex, unsigned char byteSubIndex,
                                                          unsigned int uintSize, int intTimeout, bool bUsedForRead,
                                                          unsigned int *uintSDOHandle);

/**
 * @brief Creates a raw SDO request for complete access (e.g., large data transfers).
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSlaveIndex Zero-based index of the target slave.
 * @param uintIndex Index of the object dictionary entry.
 * @param uintSize Total size of the data to transfer.
 * @param intTimeout Operation timeout in milliseconds.
 * @param bUsedForRead If true, prepares for reading; otherwise for writing.
 * @param uintSDOHandle Output parameter that receives the SDO handle.
 * @return true if the SDO request was created successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_create_raw_sdo_request_complete(unsigned int uintMasterIndex,
                                                                   unsigned int uintSlaveIndex,
                                                                   unsigned short uintIndex, unsigned int uintSize,
                                                                   int intTimeout, bool bUsedForRead,
                                                                   unsigned int *uintSDOHandle);

/**
 * @brief Receives raw data through an SDO request.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSDOHandle Handle of the SDO request.
 * @param pData Buffer to store the received data.
 * @param uintDataCount Number of bytes to receive.
 * @param bCompleteRequest Output flag indicating whether the transfer is complete.
 * @return true if data was received successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_raw_sdo_recv(unsigned int uintMasterIndex, const unsigned int uintSDOHandle,
                                                void *pData, const unsigned int uintDataCount,
                                                bool *bCompleteRequest);

/**
 * @brief Sends raw data through an SDO request.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSDOHandle Handle of the SDO request.
 * @param pData Buffer containing the data to send.
 * @param uintDataCount Number of bytes to send.
 * @param bCompleteRequest Output flag indicating whether the transfer is complete.
 * @return true if data was sent successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_raw_sdo_send(unsigned int uintMasterIndex, const unsigned int uintSDOHandle,
                                                const void *pData, const unsigned int uintDataCount,
                                                bool *bCompleteRequest);

/**
 * @brief Creates an SDO request with type-specific handling.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSlaveIndex Zero-based index of the target slave.
 * @param uintIndex Index of the object dictionary entry.
 * @param byteSubIndex Sub-index of the object dictionary entry.
 * @param intDataType Type of data being accessed (see [ETHERCAT_DATA_TYPE](file://d:\Development\Source\Ethercat\Core\EthercatDriver\EthercatDef.h#L21-L37)).
 * @param intTimeout Operation timeout in milliseconds.
 * @param bUsedForRead If true, prepares for reading; otherwise for writing.
 * @param uintSDOHandle Output parameter that receives the SDO handle.
 * @return true if the SDO request was created successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_create_sdo_request(unsigned int uintMasterIndex, unsigned int uintSlaveIndex,
                                                      unsigned short uintIndex, unsigned char byteSubIndex,
                                                      ETHERCAT_DATA_TYPE intDataType, int intTimeout,
                                                      bool bUsedForRead, unsigned int *uintSDOHandle);

/**
 * @brief Creates an extended SDO request for multi-element transfers.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSlaveIndex Zero-based index of the target slave.
 * @param uintIndex Index of the object dictionary entry.
 * @param byteSubIndex Sub-index of the object dictionary entry.
 * @param intDataType Type of data being accessed.
 * @param uintDataCount Number of elements to transfer.
 * @param intTimeout Operation timeout in milliseconds.
 * @param bUsedForRead If true, prepares for reading; otherwise for writing.
 * @param uintSDOHandle Output parameter that receives the SDO handle.
 * @return true if the SDO request was created successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_create_sdo_request_ex(unsigned int uintMasterIndex, unsigned int uintSlaveIndex,
                                                         unsigned short uintIndex, unsigned char byteSubIndex,
                                                         ETHERCAT_DATA_TYPE intDataType,
                                                         const unsigned int uintDataCount, int intTimeout,
                                                         bool bUsedForRead, unsigned int *uintSDOHandle);

/**
 * @brief Destroys an existing SDO request.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSDOHandle Handle of the SDO request to destroy.
 * @param bUsedForRead Specifies whether the SDO was used for reading.
 */
ETHERCATDRIVER_API void igh_master_destroy_sdo_request(unsigned int uintMasterIndex, unsigned int uintSDOHandle,
                                                       bool bUsedForRead);

/**
 * @brief Reads data from an SDO request synchronously.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSDOHandle Handle of the SDO request.
 * @param dblValue Output buffer that receives the double-precision floating point value.
 * @param bCompleteRequest Output flag indicating whether the transfer is complete.
 * @return true if data was read successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_sdo_recv(unsigned int uintMasterIndex, const unsigned int uintSDOHandle,
                                            double *dblValue, bool *bCompleteRequest);

/**
 * @brief Reads multiple values from an SDO request.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSDOHandle Handle of the SDO request.
 * @param dblValue Array to receive the double-precision floating point values.
 * @param uintDataCount Number of values to read.
 * @param bCompleteRequest Output flag indicating whether the transfer is complete.
 * @return true if all values were read successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_sdo_recv_ex(unsigned int uintMasterIndex, const unsigned int uintSDOHandle,
                                               double *dblValue, const unsigned int uintDataCount,
                                               bool *bCompleteRequest);

/**
 * @brief Writes data to an SDO request synchronously.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSDOHandle Handle of the SDO request.
 * @param dblValue Double-precision floating point value to write.
 * @param bCompleteRequest Output flag indicating whether the transfer is complete.
 * @return true if data was written successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_sdo_send(unsigned int uintMasterIndex, const unsigned int uintSDOHandle,
                                            const double dblValue, bool *bCompleteRequest);

/**
 * @brief Writes an array of values to an SDO request.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSDOHandle Handle of the SDO request.
 * @param dblValue Array of double-precision floating point values to write.
 * @param uintDataCount Number of values to write.
 * @param bCompleteRequest Output flag indicating whether the transfer is complete.
 * @return true if all values were written successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_sdo_send_ex(unsigned int uintMasterIndex, const unsigned int uintSDOHandle,
                                               const double *dblValue, const unsigned int uintDataCount,
                                               bool *bCompleteRequest);

/**
 * @brief Processes an SDO request immediately without waiting for the next communication cycle.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSDOHandle Handle of the SDO request to process.
 * @return true if the request was processed successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_process_sdo_immediately(unsigned int uintMasterIndex,
                                                           const unsigned int uintSDOHandle);

/**
 * @brief Requests termination of the master thread.
 *
 * @param uintMasterIndex Index of the master.
 * @return true if the termination request was accepted; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_request_terminate(unsigned int uintMasterIndex);

/**
 * @brief Checks if a termination request has been made for the master thread.
 *
 * @param uintMasterIndex Index of the master.
 * @return true if termination was requested; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_is_request_terminate(unsigned int uintMasterIndex);

/**
 * @brief Clears any active errors on the master.
 *
 * @param uintMasterIndex Index of the master.
 * @return true if errors were cleared successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_clear_error(unsigned int uintMasterIndex);

/**
 * @brief Retrieves the last error code for the specified master.
 *
 * @param uintMasterIndex Index of the master.
 * @param intErrorCode Output parameter that receives the error code.
 * @return true if the error code was retrieved successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_get_last_error(unsigned int uintMasterIndex, int &intErrorCode);

/**
 * @brief Enables or disables DC (Distributed Clocks) debug mode.
 *
 * @param uintMasterIndex Index of the master.
 * @param bEnable true to enable debug mode; false to disable.
 * @return true if the operation was successful; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_set_dc_debug_enable(unsigned int uintMasterIndex, bool bEnable);

/**
 * @brief Reads data from an ESC (EtherCAT Slave Controller) register.
 *
 * > **Note:** This function should not be used in real-time loops.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSlaveIndex Zero-based index of the target slave.
 * @param uiAddress Register address to read from.
 * @param pData Buffer to receive the register data.
 * @param uintSize Number of bytes to read.
 * @return true if the register data was read successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_read_esc_reg(unsigned int uintMasterIndex, unsigned int uintSlaveIndex,
                                                unsigned short uiAddress, void *pData, unsigned int uintSize);

/**
 * @brief Writes data to an ESC (EtherCAT Slave Controller) register.
 *
 * > **Note:** This function should not be used in real-time loops.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSlaveIndex Zero-based index of the target slave.
 * @param uiAddress Register address to write to.
 * @param pData Buffer containing the data to write.
 * @param uintSize Number of bytes to write.
 * @return true if the register was written successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_write_esc_reg(unsigned int uintMasterIndex, unsigned int uintSlaveIndex,
                                                 unsigned short uiAddress, void *pData, unsigned int uintSize);

/**
 * @brief Clears error registers on a specific slave's ESC.
 *
 * > **Note:** This function should not be used in real-time loops.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSlaveIndex Zero-based index of the target slave.
 * @return true if the error registers were cleared successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_clear_esc_error_regs(unsigned int uintMasterIndex, unsigned int uintSlaveIndex);

/**
 * @brief Clears error registers on all slaves' ESCs.
 *
 * > **Note:** This function should not be used in real-time loops.
 *
 * @param uintMasterIndex Index of the master.
 * @return true if all error registers were cleared successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_clear_all_esc_error_regs(unsigned int uintMasterIndex);

/**
 * @brief Outputs communication diagnostic information.
 *
 * > **Note:** This function should not be used in real-time loops.
 *
 * @param uintMasterIndex Index of the master.
 * @param strTitleName Title string for the diagnostic output.
 * @param info Pointer to [LPETHERCAT_COMM_DIAG_INFO](file://d:\Development\Source\Ethercat\Core\EthercatDriver\EthercatDef.h#L105-L110) structure.
 * @return true if the diagnostics were output successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_output_comm_diag_info(unsigned int uintMasterIndex, const char *strTitleName,
                                                         const LPETHERCAT_COMM_DIAG_INFO info);

/**
 * @brief Retrieves communication diagnostic information.
 *
 * > **Note:** This function should not be used in real-time loops.
 *
 * @param uintMasterIndex Index of the master.
 * @param index Diagnostic data index ([COMM_DIAG_INDEX](file://d:\Development\Source\Ethercat\Core\EthercatDriver\EthercatDef.h#L112-L119).
 * @param info Pointer to [LPETHERCAT_COMM_DIAG_INFO](file://d:\Development\Source\Ethercat\Core\EthercatDriver\EthercatDef.h#L105-L110) structure.
 * @return true if the diagnostic info was retrieved successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_get_comm_diag_info(unsigned int uintMasterIndex, const COMM_DIAG_INDEX index,
                                                      LPETHERCAT_COMM_DIAG_INFO info);

/**
 * @brief Retrieves the slot number of a specific slave.
 *
 * > **Note:** This function should not be used in real-time loops.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSlaveIndex Zero-based index of the target slave.
 * @param uintSlotNumber Output parameter that receives the slot number.
 * @return true if the slot number was retrieved successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_get_slot_number(unsigned int uintMasterIndex, unsigned int uintSlaveIndex,
                                                   unsigned int *uintSlotNumber);

/**
 * @brief Retrieves the index increment value for slots.
 *
 * > **Note:** This function should not be used in real-time loops.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSlaveIndex Zero-based index of the target slave.
 * @param uintSlotIndexIncrement Output parameter that receives the index increment value.
 * @return true if the value was retrieved successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_get_slot_index_increment(unsigned int uintMasterIndex, unsigned int uintSlaveIndex,
                                                            unsigned int *uintSlotIndexIncrement);

/**
 * @brief Retrieves the PDO (Process Data Object) count increment for slots.
 *
 * > **Note:** This function should not be used in real-time loops.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSlaveIndex Zero-based index of the target slave.
 * @param uintSlotPdoIncrement Output parameter that receives the PDO increment value.
 * @return true if the value was retrieved successfully; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_get_slot_pdo_increment(unsigned int uintMasterIndex, unsigned int uintSlaveIndex,
                                                          unsigned int *uintSlotPdoIncrement);

/**
 * @brief Enables or disables EoE (Ethernet over EtherCAT) functionality on a slave.
 *
 * > **Note:** This function should not be used in real-time loops.
 *
 * @param uintMasterIndex Index of the master.
 * @param uintSlaveIndex Zero-based index of the target slave.
 * @param bEnabled true to enable EoE; false to disable.
 * @return true if the operation was successful; false otherwise.
 */
ETHERCATDRIVER_API bool igh_master_enable_eoe_function(unsigned int uintMasterIndex, unsigned int uintSlaveIndex,
                                                       bool bEnabled);

/*the below function is only used for UI.*/
