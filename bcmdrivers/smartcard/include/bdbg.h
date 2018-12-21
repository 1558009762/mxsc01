/*****************************************************************************
 * Copyright (c) 2009 Broadcom Corporation.  All rights reserved.
 *
 * This program is the proprietary software of Broadcom Corporation and/or
 * its licensors, and may only be used, duplicated, modified or distributed
 * pursuant to the terms and conditions of a separate, written license
 * agreement executed between you and Broadcom (an "Authorized License").
 * Except as set forth in an Authorized License, Broadcom grants no license
 * (express or implied), right to use, or waiver of any kind with respect to
 * the Software, and Broadcom expressly reserves all rights in and to the
 * Software and all intellectual property rights therein.  IF YOU HAVE NO
 * AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO USE THIS SOFTWARE IN ANY
 * WAY, AND SHOULD IMMEDIATELY NOTIFY BROADCOM AND DISCONTINUE ALL USE OF
 * THE SOFTWARE.
 *
 * Except as expressly set forth in the Authorized License,
 * 1. This program, including its structure, sequence and organization,
 *    constitutes the valuable trade secrets of Broadcom, and you shall use
 *    all reasonable efforts to protect the confidentiality thereof, and to
 *    use this information only in connection with your use of Broadcom
 *    integrated circuit products.
 * 2. TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED "AS IS"
 *    AND WITH ALL FAULTS AND BROADCOM MAKES NO PROMISES, REPRESENTATIONS OR
 *    WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR OTHERWISE, WITH
 *    RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY DISCLAIMS ANY AND ALL
 *    IMPLIED WARRANTIES OF TITLE, MERCHANTABILITY, NONINFRINGEMENT, FITNESS
 *    FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR COMPLETENESS,
 *    QUIET ENJOYMENT, QUIET POSSESSION OR CORRESPONDENCE TO DESCRIPTION. YOU
 *    ASSUME THE ENTIRE RISK ARISING OUT OF USE OR PERFORMANCE OF THE SOFTWARE.
 * 3. TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL BROADCOM OR ITS
 *    LICENSORS BE LIABLE FOR (i) CONSEQUENTIAL, INCIDENTAL, SPECIAL, INDIRECT,
 *    OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR IN ANY WAY RELATING TO
 *    YOUR USE OF OR INABILITY TO USE THE SOFTWARE EVEN IF BROADCOM HAS BEEN
 *    ADVISED OF THE POSSIBILITY OF SUCH DAMAGES; OR (ii) ANY AMOUNT IN EXCESS
 *    OF THE AMOUNT ACTUALLY PAID FOR THE SOFTWARE ITSELF OR U.S. $1, WHICHEVER
 *    IS GREATER. THESE LIMITATIONS SHALL APPLY NOTWITHSTANDING ANY FAILURE OF
 *    ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
 *****************************************************************************/


#ifndef BDBG_H
#define BDBG_H

#include "blst_slist.h"

#ifdef __cplusplus
extern "C" {
#endif


/*================== Module Overview =====================================
Debug Interface provides console output according to pre-defined levels
with compile-time and run-time control.

Defined macros:
 o BDBG_MODULE - register module and allow using the rest of the BDBG_ macros.
 o BDBG_MSG, BDBG_WRN, BDBG_ERR - message output with 'Message', 'Warning' and 'Error' levels.
 o BDBG_ASSERT - evaluate expression (condition) and fail if it's false.

To activate debug build BDBG_DEBUG_BUILD macro should be defined to 1,
i.e. -D BDBG_DEBUG_BUILD=1 .

Implementation note:
During impementation of this itnerface for the given plaform, it should be
noted, that debug messages shall use different output channel than a normal
output from the system. For example in Posix like systems it could be
a good choice to use stderr for the debug messages. In such case the
following command could be used to redirect debug messages into the file:
application 2>debug_file

========================================================================*/


/* Runtime debug level */
typedef enum {
  BDBG_eTrace=0, /* Trace level */
  BDBG_eMsg, /* Message level */
  BDBG_eWrn,   /* Warning level */
  BDBG_eErr,   /* Error level */
  BDBG_P_eLastEntry
} BDBG_Level;

typedef void *BDBG_Instance;
/***************************************************************************
Summary:
	Initialize the debug interface before being used.

Description:StandardProgrammingEnvironment
	Initializes the debug interface. You must call this before making any
	other kernel interface call.

Input:
	<none>

Returns:
	BERR_SUCCESS - The debug interface successfully initialized.
****************************************************************************/
BERR_Code BDBG_Init(void);

/***************************************************************************
Summary:
	Releases resources allocated by the debug interface.

Description:
	Cleans up the debug interface interface. No debug interface calls can be made after this.

Input:
	<none>

Returns:
	<none>
****************************************************************************/
void BDBG_Uninit(void);

struct BDBG_DebugModuleInst;

BLST_S_HEAD(BDBG_DebugModuleInstHead, BDBG_DebugModuleInst);
typedef struct BDBG_DebugModule *BDBG_pDebugModule;

struct BDBG_DebugModule {
      BLST_S_ENTRY(BDBG_DebugModule) link;
      BLST_S_HEAD(BDBG_DebugModuleFileHead, BDBG_DebugModuleFile) files;
      BDBG_Level level;
      const char *name;

      struct BDBG_DebugModuleInstHead instances;

      struct BDBG_DebugModuleInst *instances_data; /* preallocated list of instances  */
      struct BDBG_DebugModuleInstHead avaliable_instances;
      BDBG_pDebugModule module_alloc; /* not NULL if memory was dynamically allocated */
};


typedef struct BDBG_DebugModuleFile {
    const char *name;
    BDBG_pDebugModule module; /* pointer to the module */
    BLST_S_ENTRY(BDBG_DebugModuleFile) link;
	struct BDBG_DebugModule moduleData; /* placeholder for module */
} BDBG_DebugModuleFile, *BDBG_pDebugModuleFile;

struct bdbg_obj
{
	const char *bdbg_obj_id;
	const void *bdbg_obj_self;
};


#ifdef BDBG_DEBUG_BUILD




/***************************************************************************
Summary:
	Set the instance debug level.

Description:
	Set debug level for the module instance.

Input:
	handle - the module instance
    level - new debug level

Returns:
	BERR_SUCCESS - instance has been registered
    BERR_INVALID_PARAMETER - unknown instance
****************************************************************************/
BERR_Code BDBG_SetInstanceLevel(BDBG_Instance handle, BDBG_Level level);

/***************************************************************************
Summary:
	Get the instance debug level.

Description:
	Get current debug level for the module instance.

Input:
	handle - the module instance

Output:
    level - current debug level

Returns:
	BERR_SUCCESS - instance has been registered
    BERR_INVALID_PARAMETER - unknown instance
****************************************************************************/
BERR_Code BDBG_GetInstanceLevel(BDBG_Instance handle, BDBG_Level *level);

/***************************************************************************
Summary:
	Set the instance name.

Description:
	Set name for the module instance.

Input:
	handle - the module instance
    new - name for the module instance

Returns:
	BERR_SUCCESS - instance has been registered
    BERR_INVALID_PARAMETER - unknown instance
****************************************************************************/
BERR_Code BDBG_SetInstanceName(BDBG_Instance handle, const char *name);

/***************************************************************************
Summary:
	Set the module debug level.

Description:
	Set debug level for the module.

Input:
	name - the module name
    level - new debug level

Returns:
	BERR_SUCCESS - the module debug level has been set
    BERR_INVALID_PARAMETER - unknown module
****************************************************************************/
BERR_Code BDBG_SetModuleLevel(const char *name, BDBG_Level level);

/***************************************************************************
Summary:
	Get module debug level.

Description:
	Get current debug level for the module.

Input:
	name - the module name
    level - new debug level

Returns:
	BERR_SUCCESS - the module debug level has been returned
    BERR_INVALID_PARAMETER - unknown module
****************************************************************************/
BERR_Code BDBG_GetModuleLevel(const char *name, BDBG_Level *level);

/***************************************************************************
Summary:
	Set global debug level.

Description:
	Set the global debug level.

Input:
    level - new debug level

Returns:
	BERR_SUCCESS - global debug level has been set
****************************************************************************/
BERR_Code BDBG_SetLevel(BDBG_Level level);

/***************************************************************************
Summary:
	Get the global debug level.

Description:
	Return the global debug level.

Input:
    <none>

Output:
    level - current global debug level

Returns:
	BERR_SUCCESS - current debug level has been retrieved
****************************************************************************/
BERR_Code BDBG_GetLevel(BDBG_Level *level);


#ifdef __GNUC__
#define BDBG_MODULE(module) static BDBG_DebugModuleFile __attribute__ ((__unused__)) dbg_module = { #module, NULL, {NULL}, {{NULL},{NULL},BDBG_eWrn,NULL,{NULL}, NULL,{NULL}, NULL} }
#else
#define BDBG_MODULE(module) static BDBG_DebugModuleFile dbg_module = { #module, NULL, {NULL}, {{NULL},{NULL},BDBG_eWrn,NULL,{NULL}, NULL,{NULL}, NULL} }
#endif

#define BDBG_P_PRINTMSG(level, fmt) (BDBG_P_TestAndPrintHeader(level, &dbg_module) ? BDBG_P_PrintString fmt, BDBG_P_PrintString("\n") : (void) 0)
#define BDBG_P_INSTANCE_PRINTMSG(level, instance, fmt) (BDBG_P_InstTestAndPrintHeader(level, &dbg_module, instance) ? BDBG_P_PrintString fmt , BDBG_P_PrintString("\n"): (void) 0)

#define BDBG_REGISTER_INSTANCE(handle) BDBG_P_RegisterInstance(handle, &dbg_module)
#define BDBG_UNREGISTER_INSTANCE(handle) BDBG_P_UnRegisterInstance(handle)

#define BDBG_ASSERT(expr) (expr) ? (void) 0 : BDBG_P_AssertFailed(#expr, __FILE__, __LINE__)

#define BDBG_CASSERT(expr) switch(0){case 0:case (expr):;}



#define BDBG_OBJECT_ID(name) const char bdbg_id__##name[]= "#" #name
#define BDBG_OBJECT_ID_DECLARE(name) extern const char bdbg_id__##name[]
#define BDBG_OBJECT(name) struct bdbg_obj bdbg_object_##name;
#define BDBG_OBJECT_INIT(ptr,name) BDBG_Object_Init(ptr,sizeof(*(ptr)),&(ptr)->bdbg_object_##name, bdbg_id__##name)
#define BDBG_OBJECT_DESTROY(ptr,name) BDBG_Object_Init(ptr,sizeof(*(ptr)),&(ptr)->bdbg_object_##name, bdbg_id__bdbg_invalid)
#define BDBG_OBJECT_SET(ptr,name) ptr->bdbg_object_##name.bdbg_obj_id=bdbg_id__##name
#define BDBG_OBJECT_UNSET(ptr,name) ptr->bdbg_object_##name.bdbg_obj_id=NULL
#define BDBG_OBJECT_ASSERT(ptr,name) BDBG_ASSERT(ptr && (ptr)->bdbg_object_##name.bdbg_obj_id==bdbg_id__##name)
#define BDBG_OBJECT_INIT_INST(ptr,name,inst) BDBG_Object_Init(ptr,sizeof(*(ptr)),&(ptr)->bdbg_object_##name,bdbg_id__##name+(unsigned)(inst))
#define BDBG_OBJECT_SET_INST(ptr,name,inst) ptr->bdbg_object_##name.bdbg_obj_id=(bdbg_id__##name + (unsigned)(inst))
#define BDBG_OBJECT_ASSERT_INST(ptr,name,inst) BDBG_ASSERT((ptr)->bdbg_object_##name.bdbg_obj_id==&bdbg_id__##name+(unsigned)(inst))

void BDBG_Object_Init(void *ptr, size_t size, struct bdbg_obj *obj, const char *id);

BDBG_OBJECT_ID_DECLARE(bdbg_invalid);

#if defined(BDBG_NO_TRACE)
#define BDBG_ENTER(function)
#define BDBG_LEAVE(function)
#define BDBG_MSG(format) BDBG_P_PRINTMSG(BDBG_eMsg, format)
#define BDBG_INSTANCE_MSG(instance, format) BDBG_P_INSTANCE_PRINTMSG(BDBG_eMsg, instance, format)
#define BDBG_WRN(format) BDBG_P_PRINTMSG(BDBG_eWrn, format)
#define BDBG_INSTANCE_WRN(instance, format) BDBG_P_INSTANCE_PRINTMSG(BDBG_eWrn, instance, format)
#elif defined(BDBG_NO_WRN)
#define BDBG_MSG(format) BDBG_NOP()
#define BDBG_INSTANCE_MSG(instance, format) BDBG_NOP()
#define BDBG_WRN(format) BDBG_NOP()
#define BDBG_INSTANCE_WRN(instance, format) BDBG_NOP()
#define BDBG_ENTER(function)
#define BDBG_LEAVE(function)
#elif defined(BDBG_NO_MSG)
#define BDBG_ENTER(function)
#define BDBG_LEAVE(function)
#define BDBG_MSG(format) BDBG_NOP()
#define BDBG_INSTANCE_MSG(format) BDBG_NOP()
#define BDBG_WRN(format) BDBG_P_PRINTMSG(BDBG_eMsg, format)
#define BDBG_INSTANCE_WRN(instance, format) BDBG_P_INSTANCE_PRINTMSG(BDBG_eMsg, instance, format)
#else
#define BDBG_ENTER(function) BDBG_EnterFunction(&dbg_module, #function, __FILE__, __LINE__)
#define BDBG_LEAVE(function) BDBG_LeaveFunction(&dbg_module, #function, __FILE__, __LINE__)
#define BDBG_MSG(format) BDBG_P_PRINTMSG(BDBG_eMsg, format)
#define BDBG_INSTANCE_MSG(instance, format) BDBG_P_INSTANCE_PRINTMSG(BDBG_eMsg, instance, format)
#define BDBG_WRN(format) BDBG_P_PRINTMSG(BDBG_eWrn, format)
#define BDBG_INSTANCE_WRN(instance, format) BDBG_P_INSTANCE_PRINTMSG(BDBG_eWrn, instance, format)
#endif
#define BDBG_ERR(format) BDBG_P_PRINTMSG(BDBG_eErr, format)
#define BDBG_INSTANCE_ERR(instance, format) BDBG_P_INSTANCE_PRINTMSG(BDBG_eErr, instance, format)


int BDBG_P_TestAndPrintHeader(BDBG_Level level, BDBG_pDebugModuleFile dbg_module);
int BDBG_P_InstTestAndPrintHeader(BDBG_Level level, BDBG_pDebugModuleFile dbg_module, BDBG_Instance handle);
void BDBG_P_RegisterInstance(BDBG_Instance handle, BDBG_pDebugModuleFile dbg_module);
void BDBG_P_UnRegisterInstance(BDBG_Instance handle, BDBG_pDebugModuleFile dbg_module);
void BDBG_EnterFunction(BDBG_pDebugModuleFile dbg_module, const char *function, const char *filename, unsigned LineNo);
void BDBG_LeaveFunction(BDBG_pDebugModuleFile dbg_module, const char *function, const char *filename, unsigned LineNo);


#else /* BDBG_DEBUG_BUILD */
/* stubs */

#define BDBG_MODULE(module) extern int bdbg_unused



#define BDBG_MSG(format) BDBG_NOP()
#define BDBG_INSTANCE_MSG(instance, format) BDBG_NOP()
#define BDBG_WRN(format) BDBG_NOP()
#define BDBG_INSTANCE_WRN(instance, format) BDBG_NOP()
#define BDBG_ERR(format) BDBG_NOP()
#define BDBG_INSTANCE_ERR(instance, format) BDBG_NOP()

#define BDBG_ENTER(function)
#define BDBG_LEAVE(function)

#define BDBG_REGISTER_INSTANCE(instance)
#define BDBG_UNREGISTER_INSTANCE(instance)

#define BDBG_SetLevel(level)  (BERR_SUCCESS)
#define BDBG_GetLevel(pLevel) (*(pLevel) = BDBG_eMsg, BERR_SUCCESS)
#define BDBG_SetModuleLevel(module, level)  (BERR_SUCCESS)
#define BDBG_GetModuleLevel(module, pLevel) (*(pLevel) = BDBG_eMsg, BERR_SUCCESS)
#define BDBG_SetInstanceLevel(handle, level) (BERR_SUCCESS)
#define BDBG_GetInstanceLevel(handle, pLevel) (*(pLevel) = BDBG_eMsg, BERR_SUCCESS)
#define BDBG_SetInstanceName(handle, name) (BERR_SUCCESS)
#define BDBG_ASSERT(expr) BDBG_NOP()
#define BDBG_CASSERT(expr)

#define BDBG_OBJECT_ID(name) extern const char bdbg_id_unused_##name
#define BDBG_OBJECT_ID_DECLARE(name) extern const char bdbg_id_unused_decl_##name
#define BDBG_OBJECT(name)
#define BDBG_OBJECT_INIT(ptr,name) (void)ptr
#define BDBG_OBJECT_DESTROY(ptr,name) (void)ptr
#define BDBG_OBJECT_SET(ptr,name) (void)ptr
#define BDBG_OBJECT_UNSET(ptr,name) (void)ptr
#define BDBG_OBJECT_ASSERT(ptr,name)  (void)ptr
#define BDBG_OBJECT_INIT_INST(ptr,name,inst) (void)ptr
#define BDBG_OBJECT_SET_INST(ptr,name,inst) (void)ptr
#define BDBG_OBJECT_TEST_INST(ptr,name,inst) (void)ptr



#endif /* BDBG_DEBUG_BUILD */

/* Private declarations starts here */
#define BDBG_NOP() (void)0
void BDBG_P_PrintString(const char *fmt, ...);
BERR_Code BDBG_P_PrintError(const char *file, unsigned lineno, const char *error, BERR_Code error_no);
void BDBG_P_AssertFailed(const char *expr, const char *file, unsigned line);

#ifdef __cplusplus
}
#endif


#endif  /* BDBG_H */
