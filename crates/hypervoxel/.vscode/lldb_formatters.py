import lldb

def format_blockid(valobj, internal_dict):
    target_obj = None
    is_pointer = valobj.TypeIsPointerType() or valobj.GetTypeName().endswith('&')

    if is_pointer:
        target_obj = valobj.Dereference()
        if not target_obj or not target_obj.IsValid():
            return "<Error: Dereference failed>"
    else:
        target_obj = valobj # Use the object directly

    if not target_obj or not target_obj.IsValid():
            return "<Error: Invalid target object>"

    # --- Try to get value from the target_obj ---
    value = None
    error = lldb.SBError()
    found_value = False

    member_obj = target_obj.GetChildMemberWithName('__0')
    if member_obj and member_obj.IsValid():
        value = member_obj.GetValueAsUnsigned(error)
        if error.Success():
            found_value = True
        else:
            error.Clear()

    if not found_value and target_obj.GetNumChildren() > 0:
            member_obj = target_obj.GetChildAtIndex(0)
            if member_obj and member_obj.IsValid():
                child_type = member_obj.GetTypeName().lower()
                if any(t in child_type for t in ['long', 'int', 'u64']):
                    value = member_obj.GetValueAsUnsigned(error)
                    if error.Success():
                        found_value = True
                    else:
                        error.Clear()

    if found_value:
        if value == 0xFFFFFFFFFFFFFFFF: return "Id(INVALID)"
        elif value == 0: return "Id(EMPTY)"
        elif (value >> 63) & 1: # is_leaf
            index = value & 0xFFFFFFFF; gen = (value >> 32) & 0x7FFF
            return f"Id(L, i: {index:08X}, g: {gen:04X})"
        else: # is_branch
            index = value & 0xFFFFFFFF; gen = (value >> 32) & 0x7FFF; mask = (value >> 47) & 0xFF; types = (value >> 55) & 0xFF
            return f"Id(B, i: {index:08X}, g: {gen:04X}, m: {mask:02X}, t: {types:02X})"
    else:
        return None # Return None to let LLDB use default formatting if we failed

def __lldb_init_module(debugger, internal_dict):
    print("Loading custom LLDB formatters for voxelis::core::block_id::BlockId...")
    type_name = "voxelis::core::block_id::BlockId"
    function_name = "lldb_formatters.format_blockid"
    command = f'type summary add --python-function {function_name} "{type_name}"'

    print(f"Executing LLDB command: {command}")
    result = debugger.HandleCommand(command)

    if "error:" in result.lower() if result else True:
        print(f"Warning/Error executing command: {result}")
    else:
        print("Custom LLDB formatter likely added successfully!")
