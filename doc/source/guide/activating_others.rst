.. admonition:: Activating fish, Nushell, or PowerShell

   The above activation command is for the default shell environments, such as ``bash``, ``zsh``, or ``sh`` on Unix, or ``cmd`` and ``powershell`` on Windows.
   If you're using a different shell, such as ``fish`` or ``Nushell``, or if you're using PowerShell and have activation issues, you may need to use a different activation file.

   .. tab-set::
      :class: sd-width-content-min
      :sync-group: shell

      .. tab-item:: fish
         :sync: fish

         .. code-block:: fish

            source .venv/bin/activate.fish

      .. tab-item:: PowerShell
         :sync: powershell

         .. code-block:: powershell

            .venv\Scripts\activate.ps1

         Note\: Windows users may need to change their PowerShell script execution policy. See the dropdown below\:

         .. dropdown:: Script Execution Policy
            :color: secondary
            :icon: shield

            If you see an error like this:

            .. code-block:: powershell

               PS > .\.venv\Scripts\activate.ps1
               .\.venv\Scripts\activate.ps1 : File .venv\Scripts\activate.ps1 cannot be loaded because running
               scripts is disabled on this system. For more information, see about_Execution_Policies at
               https:/go.microsoft.com/fwlink/?LinkID=135170 .
               At line:1 char:1
               + .\.venv\Scripts\activate.ps1
               + ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                   + CategoryInfo          : SecurityError: (:) [], PSSecurityException
                   + FullyQualifiedErrorId : UnauthorizedAccess

            Your script execution policy may be set to ``Restricted`` (check with ``Get-ExecutionPolicy``).

            To fix this, first open an administrator terminal.
            You can do this by right-clicking on the Start button or pressing :kbd:`⊞\ Win+x` and selecting :guilabel:`Terminal (Admin)` (or similar).
            Then, run the following command:

            .. code-block:: powershell

               Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser

            You will need to close and re-open your terminal windows for this to take effect.
            

      .. tab-item:: Nushell
         :sync: nushell

         .. tab-set::
            :class: sd-width-content-min
            :sync-group: os

            .. tab-item:: :fab:`windows` Windows
               :sync: windows

               .. code-block:: powershell

                  overlay use .venv\Scripts\activate.nu

            .. tab-item:: :fab:`linux` Linux / :fab:`apple` macOS / :fab:`windows`\ :fab:`linux` WSL
               :sync: posix

               .. code-block:: bash

                  overlay use .venv/bin/activate.nu
