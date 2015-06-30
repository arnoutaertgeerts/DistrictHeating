within DistrictHeating.Pipes.BaseClasses;
model TempDecay
  "Calculates decay in temperature for given inlet, delay and boundary conditions"

  parameter Modelica.SIunits.SpecificHeatCapacity cp = 4187
    "Specific heat of fluid";
  parameter Modelica.SIunits.Density rho = 1000 "Mass density of fluid";
  parameter Modelica.SIunits.Length dh = 0.05 "Hydraulic diameter of pipe";
  parameter Modelica.SIunits.ThermalConductivity k = 0.01
    "Thermal conductivity, W/mK";

  Modelica.Blocks.Interfaces.RealInput TIn "Inlet temperature at time t-delay"
    annotation (Placement(transformation(extent={{-120,20},{-80,60}})));
  Modelica.Blocks.Interfaces.RealInput tDelay
    "Delay time for current package of fluid"
    annotation (Placement(transformation(extent={{-120,-62},{-80,-22}})));
  Modelica.Blocks.Interfaces.RealInput TBou
    "Boundary temperature - Fluid would cool down to this temperature if it were to stay long enough in pipe"
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-100})));
  Modelica.Blocks.Interfaces.RealOutput TOut "Oulet temperature"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));

equation
  TOut = TBou + (TIn - TBou)*Modelica.Math.exp(-tDelay/(cp*rho*Modelica.Constants.pi*dh^2)*k*4);

    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics));
end TempDecay;
