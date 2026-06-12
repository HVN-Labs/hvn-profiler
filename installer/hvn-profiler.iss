; ============================================================================
; HVN Profiler — Windows Installer (Inno Setup 6)
; ----------------------------------------------------------------------------
; Wraps the standalone Rust GUI (hvn-profiler.exe + bundled templates) into a
; per-user installer:  hvn-profiler-Setup-vX.Y.Z.exe
;
; The profiler is a self-contained GUI — no Python, no WSL (unlike the
; HVN-SITL installer in the superproject). This just lands the exe under
; %LOCALAPPDATA%\Programs\HVN-Profiler with Start-Menu / optional desktop
; shortcuts.
;
; Build (CI injects the real tag + staged source dir):
;   iscc /DMyAppVersion=v0.16.10 /DSourceDir=dist\hvn-profiler-windows-x64 installer\hvn-profiler.iss
; ============================================================================

#define MyAppName "HVN Profiler"
#ifndef MyAppVersion
  #define MyAppVersion "v0.16.10"
#endif
; SourceDir = staged artifact folder (exe + README + LICENSE + templates\).
; Defaults to the release.yml staging path; override with /DSourceDir for
; local builds straight from target\release.
#ifndef SourceDir
  #define SourceDir "..\dist\hvn-profiler-windows-x64"
#endif
#define MyAppPublisher "HVN Labs"
#define MyAppExeName "hvn-profiler.exe"
#define MyAppURL "https://github.com/HVN-Labs/hvn-profiler"

[Setup]
; STABLE GUID — must never change between releases, or upgrades install
; side-by-side instead of replacing. Distinct from the HVN-SITL AppId.
AppId={{C7E4A9D2-3F81-4B6C-9E2A-5D8F1B0C7A36}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}/issues
AppUpdatesURL={#MyAppURL}/releases

; Per-user install — lands in %LOCALAPPDATA%\Programs\HVN-Profiler.
DefaultDirName={localappdata}\Programs\HVN-Profiler
DefaultGroupName={#MyAppName}
DisableProgramGroupPage=auto
DisableWelcomePage=no

OutputDir=..\dist
OutputBaseFilename=hvn-profiler-Setup-{#MyAppVersion}

Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible

; No admin needed; advanced users may elevate to a system-wide install.
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
MinVersion=10.0.17763
LicenseFile={#SourceDir}\LICENSE
UninstallDisplayIcon={app}\{#MyAppExeName}

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Files]
Source: "{#SourceDir}\{#MyAppExeName}"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourceDir}\README.md";       DestDir: "{app}"; Flags: ignoreversion isreadme
Source: "{#SourceDir}\LICENSE";         DestDir: "{app}"; Flags: ignoreversion
; Bundled templates are compiled into the exe, but ship the editable copies
; alongside so operators can diff / hand-edit (mag-debug, hvn-default, ...).
Source: "{#SourceDir}\templates\*"; DestDir: "{app}\templates"; Flags: ignoreversion recursesubdirs createallsubdirs skipifsourcedoesntexist

[Icons]
Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{group}\{cm:UninstallProgram,{#MyAppName}}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#StringChange(MyAppName, '&', '&&')}}"; Flags: nowait postinstall skipifsilent
