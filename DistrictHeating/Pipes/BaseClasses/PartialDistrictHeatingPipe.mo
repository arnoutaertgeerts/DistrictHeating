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

  parameter Types.PressurePerLength dp_nominal=20
    "Nominal pressure drop/meter over the pipe";
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal=0.1;

  parameter Modelica.SIunits.ThermalConductivity lambdaG=2
    "Thermal conductivity of the ground [W/mK]";
  parameter Modelica.SIunits.ThermalConductivity lambdaI=0.026
    "Thermal conductivity of the insulation [W/mK]";
  parameter Modelica.SIunits.ThermalConductivity lambdaGS = 14.6
    "Thermal conductivity of the ground surface [W/mK]";

  parameter Modelica.SIunits.Length Do=0.2 "Equivalent outer diameter";
  parameter Modelica.SIunits.Length Di=0.2 "Equivalent inner diameter";

  final parameter Modelica.SIunits.Length Heff=
    H + lambdaG/lambdaGS "Corrected depth";
  final parameter Real beta = lambdaG/lambdaI*Modelica.Math.log(ro/ri)
    "Dimensionless parameter describing the insulation";
  final parameter Modelica.SIunits.Length ro = Do/2 "Equivalent outer radius";
  final parameter Modelica.SIunits.Length ri = Di/2 "Equivalent inner radius";
  final parameter Modelica.SIunits.Length D = E/2
    "Half the distance between the center of the pipes";
  final parameter Modelica.SIunits.Mass m= Modelica.Constants.pi*Di*Di/4*L*rho;

  parameter Integer tau = 120 "Time constant of the temperature sensors";
  parameter Real hs "Heat loss factor for the symmetrical problem";
  parameter Real ha "Heat loss factor fot the anti-symmetrical problem";

  //Inputs
public
  Modelica.Blocks.Interfaces.RealInput Tg "Temperature of the ground"
                                annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-142}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-142})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-140},
            {100,140}}), graphics={
        Polygon(
          points={{30,22},{60,12},{30,0},{30,22}},
          smooth=Smooth.None,
          fillColor={255,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Ellipse(
          extent={{30,-92},{-30,-32}},
          lineColor={0,0,255},
          fillColor={0,0,255},
          fillPattern=FillPattern.Sphere),
        Polygon(
          points={{30,18},{52,12},{30,4},{30,18}},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.HorizontalCylinder,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(
          points={{-60,12},{56,12}},
          color={255,0,0},
          smooth=Smooth.None),
        Polygon(
          points={{-28,0},{-58,-10},{-28,-22},{-28,0}},
          smooth=Smooth.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Polygon(
          points={{-28,-4},{-50,-10},{-28,-18},{-28,-4}},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.HorizontalCylinder,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(
          points={{62,-10},{-50,-10}},
          color={0,0,255},
          smooth=Smooth.None),
        Ellipse(
          extent={{30,30},{-30,90}},
          lineColor={255,0,0},
          fillColor={255,0,0},
          fillPattern=FillPattern.Sphere)}),
                                 Diagram(coordinateSystem(extent={{-100,-140},{
            100,140}},  preserveAspectRatio=false),
                    graphics),
              Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -120},{100,120}}), graphics));
end PartialDistrictHeatingPipe;
