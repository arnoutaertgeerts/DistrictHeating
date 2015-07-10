within DistrictHeating.Pipes.BaseClasses;
model DistrictHeatingPipeParameters

  //Parameters
  parameter Modelica.SIunits.Length L=10 "Total length of the pipe";

  parameter Modelica.SIunits.Length H=2 "Buried depth of the pipe";
  parameter Modelica.SIunits.Length E=1.25*Di
    "Horizontal distance between pipes";
  parameter Modelica.SIunits.Length Do=Di "Equivalent outer diameter";
  parameter Modelica.SIunits.Length Di=0.2 "Equivalent inner diameter";
  parameter Modelica.SIunits.Length Dc=2.5*Di "Diameter of circumscribing pipe";

  parameter Modelica.SIunits.ThermalConductivity lambdaG=2
    "Thermal conductivity of the ground [W/mK]";
  parameter Modelica.SIunits.ThermalConductivity lambdaI=0.026
    "Thermal conductivity of the insulation [W/mK]";
  parameter Modelica.SIunits.ThermalConductivity lambdaGS = 14.6
    "Thermal conductivity of the ground surface [W/mK]";

end DistrictHeatingPipeParameters;
