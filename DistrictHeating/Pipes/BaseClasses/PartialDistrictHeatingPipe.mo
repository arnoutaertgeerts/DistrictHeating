within DistrictHeating.Pipes.BaseClasses;
partial model PartialDistrictHeatingPipe
  "Partial model for a district heating pipe"

  //Extensions
   extends IDEAS.Fluid.Interfaces.LumpedVolumeDeclarations(
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState);

  extends DistrictHeatingPipeParameters;
  extends IDEAS.Fluid.Interfaces.TwoPortFlowResistanceParameters(
    dp_nominal=dp_nominal_meter*L);

  extends IDEAS.Fluid.Interfaces.PartialFourPortInterface(
    redeclare final package Medium1=Medium,
    redeclare final package Medium2=Medium,
    final m1_flow_nominal=m_flow_nominal,
    final m2_flow_nominal=m_flow_nominal,
    final allowFlowReversal1=allowFlowReversal,
    final allowFlowReversal2=allowFlowReversal);

  parameter Modelica.SIunits.Density rho=1000 "Density of the medium";
  parameter Types.PressurePerLength dp_nominal_meter=20
    "Nominal pressure drop/meter over the pipe";

  parameter Modelica.SIunits.MassFlowRate m_flow_nominal=0.1;
  parameter Boolean allowFlowReversal=false annotation(Dialog(tab="Assumptions"), Evaluate=true);

  final parameter Modelica.SIunits.Mass m= Modelica.Constants.pi*Di*Di/4*L*rho;

  parameter Integer tau = 120 "Time constant of the temperature sensors";
  final parameter Real hs=baseConfiguration.hs
    "Heat loss factor for the symmetrical problem";
  final parameter Real ha=baseConfiguration.ha
    "Heat loss factor fot the anti-symmetrical problem";

  final parameter Types.ThermalResistanceLength Rs = 1/(2*Modelica.Constants.pi*lambdaI*hs);
  final parameter Types.ThermalResistanceLength Ra = 1/(2*Modelica.Constants.pi*lambdaI*ha);

  Modelica.SIunits.Power Q1;
  Modelica.SIunits.Power Q2;
  Modelica.SIunits.Power QLosses;

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
  replaceable DoublePipes.Configurations.TwinPipeSeparate baseConfiguration(
    H=H,
    E=E,
    Do=Do,
    Di=Di,
    Dc=Dc,
    lambdaG=lambdaG,
    lambdaI=lambdaI,
    lambdaGS=lambdaGS) constrainedby BaseConfiguration(
    H=H,
    E=E,
    Do=Do,
    Di=Di,
    Dc=Dc,
    lambdaG=lambdaG,
    lambdaI=lambdaI,
    lambdaGS=lambdaGS) annotation (Placement(transformation(extent={{70,108},{90,
            128}})), choicesAllMatching=true);

equation
  QLosses = Q1 + Q2;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-140},
            {100,140}}), graphics={
        Rectangle(
          extent={{-100,-20},{100,-100}},
          lineColor={175,175,175},
          fillPattern=FillPattern.Forward,
          fillColor={255,255,255}),
        Rectangle(
          extent={{-100,100},{100,20}},
          lineColor={175,175,175},
          fillPattern=FillPattern.Forward,
          fillColor={255,255,255}),
        Polygon(
          points={{30,18},{60,8},{30,-4},{30,18}},
          smooth=Smooth.None,
          fillColor={255,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Polygon(
          points={{30,14},{52,8},{30,0},{30,14}},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.HorizontalCylinder,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(
          points={{-60,8},{56,8}},
          color={255,0,0},
          smooth=Smooth.None),
        Polygon(
          points={{-30,4},{-60,-6},{-30,-18},{-30,4}},
          smooth=Smooth.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Polygon(
          points={{-30,0},{-52,-6},{-30,-14},{-30,0}},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.HorizontalCylinder,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(
          points={{60,-6},{-58,-6}},
          color={0,0,255},
          smooth=Smooth.None),
        Rectangle(
          extent={{-100,-30},{100,-90}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,127,255}),
        Rectangle(
          extent={{-100,90},{100,30}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={255,128,0})}),
                                 Diagram(coordinateSystem(extent={{-100,-140},{
            100,140}},  preserveAspectRatio=false),
                    graphics),
              Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -120},{100,120}}), graphics));
end PartialDistrictHeatingPipe;
