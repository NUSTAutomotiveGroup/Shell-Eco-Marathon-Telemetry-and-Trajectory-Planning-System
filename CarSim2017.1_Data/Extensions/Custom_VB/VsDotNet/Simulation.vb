Imports System.Runtime.InteropServices
Imports System.IO


Public Class Simulation

  '*****************************************************************
  ' VS API Function Pointers
  '*****************************************************************
  <UnmanagedFunctionPointer(CallingConvention.Cdecl)>
  Private Delegate Sub vs_run(ByVal simfile As String)
  Private vs_run_ptr As vs_run

  <UnmanagedFunctionPointer(CallingConvention.Cdecl)>
  Private Delegate Sub vs_read_configuration(ByVal Simfile As String, ByRef n_imp As Integer, _
                                             ByRef n_exp As Integer, ByRef tstart As Double, _
                                             ByRef tstop As Double, ByRef tstep As Double)
  Private vs_read_configuration_ptr As vs_read_configuration

  <UnmanagedFunctionPointer(CallingConvention.Cdecl)>
  Private Delegate Function vs_integrate_io(ByVal t As Double, ByRef import As Double, _
                                            ByRef export As Double) As Integer
  Private vs_integrate_io_ptr As vs_integrate_io

  <UnmanagedFunctionPointer(CallingConvention.Cdecl)>
  Private Delegate Sub vs_copy_export_vars(ByRef export As Double)
  Private vs_copy_export_vars_ptr As vs_copy_export_vars

  <UnmanagedFunctionPointer(CallingConvention.Cdecl)>
  Private Delegate Sub vs_terminate_run(ByVal t As Double)
  Private vs_terminate_run_ptr As vs_terminate_run

  <UnmanagedFunctionPointer(CallingConvention.Cdecl)>
  Private Delegate Function vs_error_occurred() As Integer
  Private vs_error_occurred_ptr As vs_error_occurred

  <UnmanagedFunctionPointer(CallingConvention.Cdecl)>
  Private Delegate Function vs_road_l(ByVal sXprevAs As Double, ByVal Ypreview As Double) As Double
  Private vs_road_l_ptr As vs_road_l

  '*****************************************************************
  ' DLL loading and unloading API functions
  '*****************************************************************
  Private Declare Function LoadLibraryA Lib "kernel32.dll" (ByVal libname As String) As IntPtr
  Private Declare Function GetProcAddress Lib "kernel32.dll" (ByVal hModule As IntPtr, ByVal procName As String) As IntPtr
  Private Declare Function FreeLibrary Lib "kernel32.dll" (ByVal hModule As IntPtr) As Boolean

  Public Function get_api(dllHandle As IntPtr) As Boolean
    Dim got_api As Boolean = False

    If dllHandle <> IntPtr.Zero Then
      Me.vs_copy_export_vars_ptr = Marshal.GetDelegateForFunctionPointer(GetProcAddress(dllHandle, "vs_copy_export_vars"), GetType(vs_copy_export_vars))
      Me.vs_error_occurred_ptr = Marshal.GetDelegateForFunctionPointer(GetProcAddress(dllHandle, "vs_error_occurred"), GetType(vs_error_occurred))
      Me.vs_integrate_io_ptr = Marshal.GetDelegateForFunctionPointer(GetProcAddress(dllHandle, "vs_integrate_io"), GetType(vs_integrate_io))
      Me.vs_read_configuration_ptr = Marshal.GetDelegateForFunctionPointer(GetProcAddress(dllHandle, "vs_read_configuration"), GetType(vs_read_configuration))
      Me.vs_road_l_ptr = Marshal.GetDelegateForFunctionPointer(GetProcAddress(dllHandle, "vs_road_l"), GetType(vs_road_l))
      Me.vs_run_ptr = Marshal.GetDelegateForFunctionPointer(GetProcAddress(dllHandle, "vs_run"), GetType(vs_run))
      Me.vs_terminate_run_ptr = Marshal.GetDelegateForFunctionPointer(GetProcAddress(dllHandle, "vs_terminate_run"), GetType(vs_terminate_run))

      got_api = True
    End If

    Return got_api
  End Function

  '*****************************************************************
  ' Simfile parsing
  '*****************************************************************
  Private Function GetParameterKey(ByVal paramline As String) As String
    Dim pos As Integer = paramline.IndexOf(" ")
    If pos >= 0 Then
      Return paramline.Substring(0, pos).Trim().ToUpper()
    Else
      Return paramline.Trim()
    End If
  End Function

  Private Function GetParameterValue(ByVal paramline As String) As String
    Dim pos As Integer = paramline.IndexOf(" ")
    If pos >= 0 Then
      Return paramline.Substring(pos).Trim()
    Else
      Return ""
    End If
  End Function

  Public Function get_dll_path(ByVal pathToSimFile As String) As String
    Dim dllpath As String = Nothing
    Dim progdir As String = ""
    Dim vehcode As String = ""

    Using fs As New StreamReader(pathToSimFile)
      Dim line As String = fs.ReadLine()
      While Not line Is Nothing
        Select Case Me.GetParameterKey(line)
          Case "DLLFILE"
            dllpath = Me.GetParameterValue(line)
          Case "VEHICLE_CODE"
            vehcode = Me.GetParameterValue(line)
          Case "PROGDIR"
            progdir = Me.GetParameterValue(line)
        End Select
        line = fs.ReadLine()
      End While
      fs.Close()
    End Using

    If dllpath Is Nothing Then
      Dim dlldir As String
      If Environment.Is64BitProcess Then
        dlldir = "Default64"
      Else
        dlldir = "Default"
      End If
      dllpath = Path.Combine(progdir, "Programs", "Solvers", dlldir, vehcode + ".dll")
    End If
    Return dllpath
  End Function

  Public Function LoadLibrary(ByVal pathToVehicleSimDLL As String) As IntPtr
    Dim dllhandle As IntPtr = IntPtr.Zero
    If File.Exists(pathToVehicleSimDLL) Then
      dllhandle = LoadLibraryA(pathToVehicleSimDLL)
      If dllhandle = IntPtr.Zero Then
        Throw New ApplicationException("The DLL specified in the simfile could not be loaded: " + pathToVehicleSimDLL)
      End If
    End If
    Return dllhandle
  End Function

  Public Sub UnloadLibrary(ByVal dllHandle As IntPtr)
    If dllhandle <> IntPtr.Zero Then
      FreeLibrary(dllhandle)
    End If
  End Sub

  '************************************************************
  ' VS API Implementations
  '************************************************************
  Public Sub run(ByVal pathToSimFile As String)
    vs_run_ptr(pathToSimFile)
  End Sub

  Public Sub ReadConfiguration(ByVal pathToSimfile As String, ByRef n_import As Integer, ByRef n_export As Integer, _
                               ByRef t_start As Double, ByRef t_stop As Double, ByRef t_step As Double)
    vs_read_configuration_ptr(pathToSimfile, n_import, n_export, t_start, t_stop, t_step)
  End Sub

  Public Sub CopyExportVars(ByRef export As Double)
    vs_copy_export_vars_ptr(export)
  End Sub

  Public Function GetRoadL(ByVal x As Double, ByVal y As Double) As Double
    Return vs_road_l_ptr(x, y)
  End Function

  Public Function IntegrateIO(ByVal t_current As Double, ByRef import As Double, ByRef export As Double) As Integer
    Return vs_integrate_io_ptr(t_current, import, export)
  End Function

  Public Sub TerminateRun(ByVal t As Double)
    vs_terminate_run_ptr(t)
  End Sub

End Class
