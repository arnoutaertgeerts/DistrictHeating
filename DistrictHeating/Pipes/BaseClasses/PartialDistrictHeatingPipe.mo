within DistrictHeating.Pipes.BaseClasses;
partial model PartialDistrictHeatingPipe
  "Partial model for a district heating pipe"

  //Extensions
  extends IDEAS.Fluid.Interfaces.LumpedVolumeDeclarations(
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState);

  extends IDEAS.Fluid.Interfaces.PartialFourPortInterface(
    redeclare final package Medium1=Medium,
    redeclare final package Medium2=Medium,
    m1_flow_nominal=m_flow_nominal,
    m2_flow_nominal=m_flow_nominal);

  //Parameters
  parameter Modelica.SIunits.Length L=10 "Total length of the pipe";
  parameter Modelica.SIunits.Density rho=1000 "Density of the medium";

  parameter Modelica.SIunits.Length H=2 "Buried depth of the pipe";
  parameter Modelica.SIunits.Length E=1.25*Di
    "Horizontal distance between pipes";
  parameter Modelica.SIunits.Length Do=0.2 "Equivalent outer diameter";
  parameter Modelica.SIunits.Length Di=0.2 "Equivalent inner diameter";
  parameter Modelica.SIunits.Length Dc=2.5*Di "Diameter of circumscribing pipe";

  parameter Types.PressurePerLength dp_nominal=20
    "Nominal pressure drop/meter over the pipe";
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal=0.1;

  parameter Modelica.SIunits.ThermalConductivity lambdaG=2
    "Thermal conductivity of the ground [W/mK]";
  parameter Modelica.SIunits.ThermalConductivity lambdaI=0.026
    "Thermal conductivity of the insulation [W/mK]";
  parameter Modelica.SIunits.ThermalConductivity lambdaGS = 14.6
    "Thermal conductivity of the ground surface [W/mK]";

  final parameter Modelica.SIunits.Mass m= Modelica.Constants.pi*Di*Di/4*L*rho;

  parameter Integer tau = 120 "Time constant of the temperature sensors";
  final parameter Real hs=baseConfiguration.hs
    "Heat loss factor for the symmetrical problem";
  final parameter Real ha=baseConfiguration.ha
    "Heat loss factor fot the anti-symmetrical problem";

  final parameter Types.ThermalResistanceLength Rs = 1/(2*Modelica.Constants.pi*lambdaI*hs);
  final parameter Types.ThermalResistanceLength Ra = 1/(2*Modelica.Constants.pi*lambdaI*ha);

  //Inputs
  Modelica.Blocks.Interfaces.RealInput Tg "Temperature of the ground"
                                annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-142}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-142})));

  //Components
  replaceable Configurations.TwinPipeSeparate  baseConfiguration(
    H=H,
    E=E,
    Do=Do,
    Di=Di,
    Dc=Dc,
    lambdaG=lambdaG,
    lambdaI=lambdaI,
    lambdaGS=lambdaGS) constrainedby Configurations.BaseConfiguration(
    H=H,
    E=E,
    Do=Do,
    Di=Di,
    Dc=Dc,
    lambdaG=lambdaG,
    lambdaI=lambdaI,
    lambdaGS=lambdaGS)
    annotation (Placement(transformation(extent={{70,108},{90,128}})), choicesAllMatching=true);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-140},
            {100,140}}), graphics={
        Polygon(
          points={{32,110},{62,100},{32,88},{32,110}},
          smooth=Smooth.None,
          fillColor={255,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Polygon(
          points={{32,106},{54,100},{32,92},{32,106}},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.HorizontalCylinder,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(
          points={{-60,100},{56,100}},
          color={255,0,0},
          smooth=Smooth.None),
        Polygon(
          points={{-32,-96},{-62,-106},{-32,-118},{-32,-96}},
          smooth=Smooth.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Polygon(
          points={{-32,-100},{-54,-106},{-32,-114},{-32,-100}},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.HorizontalCylinder,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(
          points={{58,-106},{-60,-106}},
          color={0,0,255},
          smooth=Smooth.None),
        Rectangle(
          extent={{-100,-30},{100,-90}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,127,255}),
        Rectangle(
          extent={{-100,88},{100,28}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={255,128,0})}),
                                 Diagram(coordinateSystem(extent={{-100,-140},{
            100,140}},  preserveAspectRatio=false),
                    graphics),
              Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -120},{100,120}}), graphics));
end PartialDistrictHeatingPipe;
