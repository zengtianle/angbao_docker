// Copyright 2019 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Generated with protocol_gen.go -- do not edit this file.
//   go run scripts/protocol_gen/protocol_gen.go
//
// DAP version 1.59.0

#include "dap/protocol.h"

namespace dap {

DAP_IMPLEMENT_STRUCT_TYPEINFO(AttachResponse, "");

DAP_IMPLEMENT_STRUCT_TYPEINFO(BreakpointLocationsResponse,
                              "",
                              DAP_FIELD(breakpoints, "breakpoints"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(CancelResponse, "");

DAP_IMPLEMENT_STRUCT_TYPEINFO(CompletionsResponse,
                              "",
                              DAP_FIELD(targets, "targets"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(ConfigurationDoneResponse, "");

DAP_IMPLEMENT_STRUCT_TYPEINFO(ContinueResponse,
                              "",
                              DAP_FIELD(allThreadsContinued,
                                        "allThreadsContinued"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(DataBreakpointInfoResponse,
                              "",
                              DAP_FIELD(accessTypes, "accessTypes"),
                              DAP_FIELD(canPersist, "canPersist"),
                              DAP_FIELD(dataId, "dataId"),
                              DAP_FIELD(description, "description"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(DisassembleResponse,
                              "",
                              DAP_FIELD(instructions, "instructions"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(DisconnectResponse, "");

DAP_IMPLEMENT_STRUCT_TYPEINFO(ErrorResponse, "", DAP_FIELD(error, "error"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(EvaluateResponse,
                              "",
                              DAP_FIELD(indexedVariables, "indexedVariables"),
                              DAP_FIELD(memoryReference, "memoryReference"),
                              DAP_FIELD(namedVariables, "namedVariables"),
                              DAP_FIELD(presentationHint, "presentationHint"),
                              DAP_FIELD(result, "result"),
                              DAP_FIELD(type, "type"),
                              DAP_FIELD(variablesReference,
                                        "variablesReference"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(ExceptionInfoResponse,
                              "",
                              DAP_FIELD(breakMode, "breakMode"),
                              DAP_FIELD(description, "description"),
                              DAP_FIELD(details, "details"),
                              DAP_FIELD(exceptionId, "exceptionId"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(GotoResponse, "");

DAP_IMPLEMENT_STRUCT_TYPEINFO(GotoTargetsResponse,
                              "",
                              DAP_FIELD(targets, "targets"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(
    InitializeResponse,
    "",
    DAP_FIELD(additionalModuleColumns, "additionalModuleColumns"),
    DAP_FIELD(completionTriggerCharacters, "completionTriggerCharacters"),
    DAP_FIELD(exceptionBreakpointFilters, "exceptionBreakpointFilters"),
    DAP_FIELD(supportSuspendDebuggee, "supportSuspendDebuggee"),
    DAP_FIELD(supportTerminateDebuggee, "supportTerminateDebuggee"),
    DAP_FIELD(supportedChecksumAlgorithms, "supportedChecksumAlgorithms"),
    DAP_FIELD(supportsBreakpointLocationsRequest,
              "supportsBreakpointLocationsRequest"),
    DAP_FIELD(supportsCancelRequest, "supportsCancelRequest"),
    DAP_FIELD(supportsClipboardContext, "supportsClipboardContext"),
    DAP_FIELD(supportsCompletionsRequest, "supportsCompletionsRequest"),
    DAP_FIELD(supportsConditionalBreakpoints, "supportsConditionalBreakpoints"),
    DAP_FIELD(supportsConfigurationDoneRequest,
              "supportsConfigurationDoneRequest"),
    DAP_FIELD(supportsDataBreakpoints, "supportsDataBreakpoints"),
    DAP_FIELD(supportsDelayedStackTraceLoading,
              "supportsDelayedStackTraceLoading"),
    DAP_FIELD(supportsDisassembleRequest, "supportsDisassembleRequest"),
    DAP_FIELD(supportsEvaluateForHovers, "supportsEvaluateForHovers"),
    DAP_FIELD(supportsExceptionFilterOptions, "supportsExceptionFilterOptions"),
    DAP_FIELD(supportsExceptionInfoRequest, "supportsExceptionInfoRequest"),
    DAP_FIELD(supportsExceptionOptions, "supportsExceptionOptions"),
    DAP_FIELD(supportsFunctionBreakpoints, "supportsFunctionBreakpoints"),
    DAP_FIELD(supportsGotoTargetsRequest, "supportsGotoTargetsRequest"),
    DAP_FIELD(supportsHitConditionalBreakpoints,
              "supportsHitConditionalBreakpoints"),
    DAP_FIELD(supportsInstructionBreakpoints, "supportsInstructionBreakpoints"),
    DAP_FIELD(supportsLoadedSourcesRequest, "supportsLoadedSourcesRequest"),
    DAP_FIELD(supportsLogPoints, "supportsLogPoints"),
    DAP_FIELD(supportsModulesRequest, "supportsModulesRequest"),
    DAP_FIELD(supportsReadMemoryRequest, "supportsReadMemoryRequest"),
    DAP_FIELD(supportsRestartFrame, "supportsRestartFrame"),
    DAP_FIELD(supportsRestartRequest, "supportsRestartRequest"),
    DAP_FIELD(supportsSetExpression, "supportsSetExpression"),
    DAP_FIELD(supportsSetVariable, "supportsSetVariable"),
    DAP_FIELD(supportsSingleThreadExecutionRequests,
              "supportsSingleThreadExecutionRequests"),
    DAP_FIELD(supportsStepBack, "supportsStepBack"),
    DAP_FIELD(supportsStepInTargetsRequest, "supportsStepInTargetsRequest"),
    DAP_FIELD(supportsSteppingGranularity, "supportsSteppingGranularity"),
    DAP_FIELD(supportsTerminateRequest, "supportsTerminateRequest"),
    DAP_FIELD(supportsTerminateThreadsRequest,
              "supportsTerminateThreadsRequest"),
    DAP_FIELD(supportsValueFormattingOptions, "supportsValueFormattingOptions"),
    DAP_FIELD(supportsWriteMemoryRequest, "supportsWriteMemoryRequest"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(LaunchResponse, "");

DAP_IMPLEMENT_STRUCT_TYPEINFO(LoadedSourcesResponse,
                              "",
                              DAP_FIELD(sources, "sources"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(ModulesResponse,
                              "",
                              DAP_FIELD(modules, "modules"),
                              DAP_FIELD(totalModules, "totalModules"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(NextResponse, "");

DAP_IMPLEMENT_STRUCT_TYPEINFO(PauseResponse, "");

DAP_IMPLEMENT_STRUCT_TYPEINFO(ReadMemoryResponse,
                              "",
                              DAP_FIELD(address, "address"),
                              DAP_FIELD(data, "data"),
                              DAP_FIELD(unreadableBytes, "unreadableBytes"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(RestartFrameResponse, "");

DAP_IMPLEMENT_STRUCT_TYPEINFO(RestartResponse, "");

DAP_IMPLEMENT_STRUCT_TYPEINFO(ReverseContinueResponse, "");

DAP_IMPLEMENT_STRUCT_TYPEINFO(RunInTerminalResponse,
                              "",
                              DAP_FIELD(processId, "processId"),
                              DAP_FIELD(shellProcessId, "shellProcessId"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(ScopesResponse, "", DAP_FIELD(scopes, "scopes"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(SetBreakpointsResponse,
                              "",
                              DAP_FIELD(breakpoints, "breakpoints"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(SetDataBreakpointsResponse,
                              "",
                              DAP_FIELD(breakpoints, "breakpoints"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(SetExceptionBreakpointsResponse,
                              "",
                              DAP_FIELD(breakpoints, "breakpoints"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(SetExpressionResponse,
                              "",
                              DAP_FIELD(indexedVariables, "indexedVariables"),
                              DAP_FIELD(namedVariables, "namedVariables"),
                              DAP_FIELD(presentationHint, "presentationHint"),
                              DAP_FIELD(type, "type"),
                              DAP_FIELD(value, "value"),
                              DAP_FIELD(variablesReference,
                                        "variablesReference"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(SetFunctionBreakpointsResponse,
                              "",
                              DAP_FIELD(breakpoints, "breakpoints"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(SetInstructionBreakpointsResponse,
                              "",
                              DAP_FIELD(breakpoints, "breakpoints"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(SetVariableResponse,
                              "",
                              DAP_FIELD(indexedVariables, "indexedVariables"),
                              DAP_FIELD(namedVariables, "namedVariables"),
                              DAP_FIELD(type, "type"),
                              DAP_FIELD(value, "value"),
                              DAP_FIELD(variablesReference,
                                        "variablesReference"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(SourceResponse,
                              "",
                              DAP_FIELD(content, "content"),
                              DAP_FIELD(mimeType, "mimeType"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(StackTraceResponse,
                              "",
                              DAP_FIELD(stackFrames, "stackFrames"),
                              DAP_FIELD(totalFrames, "totalFrames"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(StartDebuggingResponse, "");

DAP_IMPLEMENT_STRUCT_TYPEINFO(StepBackResponse, "");

DAP_IMPLEMENT_STRUCT_TYPEINFO(StepInResponse, "");

DAP_IMPLEMENT_STRUCT_TYPEINFO(StepInTargetsResponse,
                              "",
                              DAP_FIELD(targets, "targets"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(StepOutResponse, "");

DAP_IMPLEMENT_STRUCT_TYPEINFO(TerminateResponse, "");

DAP_IMPLEMENT_STRUCT_TYPEINFO(TerminateThreadsResponse, "");

DAP_IMPLEMENT_STRUCT_TYPEINFO(ThreadsResponse,
                              "",
                              DAP_FIELD(threads, "threads"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(VariablesResponse,
                              "",
                              DAP_FIELD(variables, "variables"));

DAP_IMPLEMENT_STRUCT_TYPEINFO(WriteMemoryResponse,
                              "",
                              DAP_FIELD(bytesWritten, "bytesWritten"),
                              DAP_FIELD(offset, "offset"));

}  // namespace dap
