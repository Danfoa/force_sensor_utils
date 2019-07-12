# Constants -----------------------------
# Force and Torque axis labels 
string FX = "Fx"
string FY = "Fy"
string FZ = "Fz"
string EQUIVALENT_FORCE = "F"
string MX = "Mx"
string MY = "My"
string MZ = "Mz"
string EQUIVALENT_TORQUE = "M"
# Value indicating that a given threshold should not be 
# taken into consideration while monitoring 
float32 DISABLED = -999
# ----------------------------------------

# Force/Torque axis label 
string axis_name      # e.g. AxisRange.EQUIVALENT_FORCE
# Upper threshold
float32 max_value     #[N] or [Nm] 
# Lower threshold
float32 min_value     #[N] or [Nm] 
# Current axis value 
float32 current_value #[N] or [Nm] 