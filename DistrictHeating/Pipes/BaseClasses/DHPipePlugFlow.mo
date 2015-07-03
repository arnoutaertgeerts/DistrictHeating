within DistrictHeating.Pipes.BaseClasses;
partial model DHPipePlugFlow

  //Extensions
  extends IDEAS.Fluid.Interfaces.LumpedVolumeDeclarations(
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState);

  extends IDEAS.Fluid.Interfaces.PartialFourPortInterface(
    redeclare final package Medium1=Medium,
    redeclare final package Medium2=Medium,
    m1_flow_nominal=m_flow_nominal,
    m2_flow_nominal=m_flow_nominal);

  //Parameters
  parameter Real hs;
  parameter Real ha;

  parameter Modelica.SIunits.Length L=10 "Total length of the pipe";
  parameter Modelica.SIunits.Density rho=1000 "Density of the medium";

  parameter Modelica.SIunits.ThermalConductivity lambdaG=2
    "Thermal conductivity of the ground [W/mK]";
  parameter Modelica.SIunits.ThermalConductivity lambdaI=0.026
    "Thermal conductivity of the insulation [W/mK]";
  parameter Modelica.SIunits.ThermalConductivity lambdaGS = 14.6
    "Thermal conductivity of the ground surface [W/mK]";

  parameter Modelica.SIunits.Length H=2 "Buried depth of the pipe";
  parameter Modelica.SIunits.Length E=1.25*Di
    "Horizontal distance between pipes";
  parameter Modelica.SIunits.Length Do=0.2 "Equivalent outer diameter";
  parameter Modelica.SIunits.Length Di=0.2 "Equivalent inner diameter";

  final parameter Modelica.SIunits.Length Heff=H + lambdaG/lambdaGS
    "Corrected depth";
  final parameter Real beta = lambdaG/lambdaI*Modelica.Math.log(ro/ri)
    "Dimensionless parameter describing the insulation";
  final parameter Modelica.SIunits.Length ro = Do/2 "Equivalent outer radius";
  final parameter Modelica.SIunits.Length ri = Di/2 "Equivalent inner radius";
  final parameter Modelica.SIunits.Length D = E/2
    "Half the distance between the center of the pipes";
  final parameter Modelica.SIunits.Mass m=Modelica.Constants.pi*Di*Di/4*L*rho;

  final parameter Real a = (1/hs-1/ha)/(1/hs+1/ha);
  final parameter Real b = (2/ha)/(1/hs+1/ha);

  parameter Types.PressurePerLength dp_nominal=20
    "Nominal pressure drop/meter over the pipe";
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal=0.1;

  //Variables
  Modelica.SIunits.Temperature T1Bou;
  Modelica.SIunits.Temperature T2Bou;

  Modelica.SIunits.Temperature T1Avg;
  Modelica.SIunits.Temperature T2Avg;

  Modelica.SIunits.Power Q1 "Heat losses from the supply line";
  Modelica.SIunits.Power Q2 "Heat losses from the return line";

  //Interfaces
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
  PlugFlowHeatLosses plugFlow1(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    L=L,
    D=Di,
    S=Modelica.Constants.pi*(ha + hs)/2)
    annotation (Placement(transformation(extent={{-10,50},{10,70}})));
  PlugFlowHeatLosses plugFlow2(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    L=L,
    D=Di,
    rho=rho,
    S=Modelica.Constants.pi*(ha + hs)/2)
    annotation (Placement(transformation(extent={{10,-50},{-10,-70}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T2In(
    redeclare package Medium = Medium, m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{-20,-70},{-40,-50}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T2Out(
    redeclare package Medium=Medium, m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{40,-70},{20,-50}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T1Out(
    redeclare package Medium=Medium, m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{20,50},{40,70}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T1In(
    redeclare package Medium = Medium, m_flow_nominal=m_flow_nominal)
                                    annotation (Placement(transformation(extent={{-40,50},{-20,70}})));
public
  Modelica.Blocks.Sources.RealExpression SupplyBoundaryTemperature(y=T1Bou)
    annotation (Placement(transformation(extent={{-32,90},{-12,110}})));
  Modelica.Blocks.Sources.RealExpression ReturnBoundaryTemperature(y=T2Bou)
    annotation (Placement(transformation(extent={{-40,-110},{-20,-90}})));
equation
  T1Avg = (T1In.T + T1Out.T)/2;
  T2Avg = (T2In.T + T2Out.T)/2;

  T1Bou = T2Avg*a + Tg*b;
  T2Bou = T1Avg*a + Tg*b;

  Q1 = plugFlow1.Q_Losses;
  Q2 = plugFlow2.Q_Losses;

  connect(plugFlow2.port_b, T2In.port_a) annotation (Line(
      points={{-10,-60},{-20,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T2Out.port_b, plugFlow2.port_a) annotation (Line(
      points={{20,-60},{10,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T2In.port_b, port_b2) annotation (Line(
      points={{-40,-60},{-100,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T2Out.port_a, port_a2) annotation (Line(
      points={{40,-60},{100,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(plugFlow1.port_b, T1Out.port_a) annotation (Line(
      points={{10,60},{20,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T1Out.port_b, port_b1) annotation (Line(
      points={{40,60},{100,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T1In.port_b, plugFlow1.port_a) annotation (Line(
      points={{-20,60},{-10,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(port_a1, T1In.port_a) annotation (Line(
      points={{-100,60},{-40,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(SupplyBoundaryTemperature.y, plugFlow1.TBoundary) annotation (Line(
      points={{-11,100},{0.2,100},{0.2,65}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(ReturnBoundaryTemperature.y, plugFlow2.TBoundary) annotation (Line(
      points={{-19,-100},{-0.2,-100},{-0.2,-65}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Icon(coordinateSystem(extent={{-100,-140},{100,140}}), graphics={
                                 Text(
          extent={{-151,147},{149,107}},
          lineColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,127,255},
          textString="%name"),
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
          fillPattern=FillPattern.Sphere)}), Diagram(coordinateSystem(extent={{-100,
            -140},{100,140}}, preserveAspectRatio=false), graphics));
end DHPipePlugFlow;
