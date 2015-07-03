within DistrictHeating.Pipes.BaseClasses;
partial model DistrictHeatingPipeTR
  "A partial for a return and supply district heating pipe model based on Kvisgaard and Hadvig (1980)"

  //Extensions
  extends IDEAS.Fluid.Interfaces.LumpedVolumeDeclarations(
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState);

  extends IDEAS.Fluid.Interfaces.PartialFourPortInterface(
    redeclare final package Medium1=Medium,
    redeclare final package Medium2=Medium,
    m1_flow_nominal=m_flow_nominal,
    m2_flow_nominal=m_flow_nominal);

  //Parameters
  parameter Integer nSeg=5;

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

  final parameter Modelica.SIunits.Length Heff=
    H + lambdaG/lambdaGS "Corrected depth";
  final parameter Real beta = lambdaG/lambdaI*Modelica.Math.log(ro/ri)
    "Dimensionless parameter describing the insulation";
  final parameter Modelica.SIunits.Length ro = Do/2 "Equivalent outer radius";
  final parameter Modelica.SIunits.Length ri = Di/2 "Equivalent inner radius";
  final parameter Modelica.SIunits.Length D = E/2
    "Half the distance between the center of the pipes";
  final parameter Modelica.SIunits.Mass m= Modelica.Constants.pi*Di*Di/4*L*rho;

  parameter Types.PressurePerLength dp_nominal=20
    "Nominal pressure drop/meter over the pipe";
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal=0.1;

  parameter Integer tau = 120 "Time constant of the temperature sensors";

  parameter Real hs "Heat loss factor for the symmetrical problem";
  parameter Real ha "Heat loss factor fot the anti-symmetrical problem";

  parameter Real Rs = 1/(2*Modelica.Constants.pi*lambdaI*hs);
  parameter Real Ra = 1/(2*Modelica.Constants.pi*lambdaI*ha);

  parameter Real R12 = (4*Ra*Rs)/(2*Rs-Ra);
  parameter Real Rbou = 2*Rs;

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

  //Variables
  Modelica.SIunits.Temperature T1;
  Modelica.SIunits.Temperature T2;

  Modelica.SIunits.Temperature Ts "Temperature of the symmetrical problem";
  Modelica.SIunits.Temperature Ta "Temperature of the asymmetrical problem";

  Modelica.SIunits.Power Q1 "Heat losses of pipe 1";
  Modelica.SIunits.Power Q2 "Heat losses of pipe 2";

  Types.PowerPerLength Qs "Symmetrical heat losses";
  Types.PowerPerLength Qa "Assymmetrical heat losses";

  //Components
  IDEAS.Fluid.Sensors.TemperatureTwoPort TIn1(
    redeclare package Medium=Medium,
    m_flow_nominal=m1_flow_nominal,
    tau=tau)
    annotation (Placement(transformation(extent={{-80,50},{-60,70}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOut1(
    redeclare package Medium=Medium,
    m_flow_nominal=m1_flow_nominal,
    tau=tau)
    annotation (Placement(transformation(extent={{60,50},{80,70}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOut2(
    redeclare package Medium=Medium,
    m_flow_nominal=m2_flow_nominal,
    tau=tau)
    annotation (Placement(transformation(extent={{-60,-70},{-80,-50}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TIn2(
    redeclare package Medium=Medium,
    m_flow_nominal=m2_flow_nominal,
    tau=tau)
    annotation (Placement(transformation(extent={{80,-70},{60,-50}})));
  Buildings.Fluid.FixedResistances.Pipe      Pipe1(
    redeclare package Medium = Medium,
    m_flow_nominal=m1_flow_nominal,
    dp_nominal=dp_nominal*L,
    massDynamics=massDynamics,
    energyDynamics=energyDynamics,
    length=L,
    thicknessIns=0.0001,
    diameter=Di,
    lambdaIns=lambdaI,
    nSeg=nSeg,
    useMultipleHeatPorts=true)
    annotation (Placement(transformation(extent={{22,50},{42,70}})));
  Buildings.Fluid.FixedResistances.Pipe      Pipe2(
    redeclare package Medium = Medium,
    massDynamics=massDynamics,
    energyDynamics=energyDynamics,
    m_flow_nominal=m2_flow_nominal,
    dp_nominal=dp_nominal*L,
    length=L,
    thicknessIns=0.00001,
    lambdaIns=lambdaI,
    diameter=Di,
    nSeg=nSeg,
    useMultipleHeatPorts=true)
    annotation (Placement(transformation(extent={{42,-50},{22,-70}})));

  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature[nSeg]
    prescribedTemperature annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,0})));
  Modelica.Thermal.HeatTransfer.Components.ThermalResistor R12m[nSeg](R=R12)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={32,0})));
  Modelica.Thermal.HeatTransfer.Components.ThermalResistor R1[nSeg](R=Rbou)
                                                                    annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,24})));
  Modelica.Thermal.HeatTransfer.Components.ThermalResistor R2[nSeg](R=Rbou)
                                                                    annotation (
     Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={0,-24})));
equation
  T1 = (TIn1.T + TOut1.T)/2;
  T2 = (TIn2.T + TOut2.T)/2;

  Ts = (T1 + T2)/2;
  Ta = (T1 - T2)/2;

  Q1 = (Qs + Qa)*L;
  Q2 = (Qs - Qa)*L;

  connect(port_a1, TIn1.port_a) annotation (Line(
      points={{-100,60},{-80,60}},
      color={0,127,255},
      smooth=Smooth.None));

  connect(TOut1.port_b, port_b1) annotation (Line(
      points={{80,60},{100,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(port_b2, TOut2.port_b) annotation (Line(
      points={{-100,-60},{-80,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TIn2.port_a, port_a2) annotation (Line(
      points={{80,-60},{100,-60}},
      color={0,127,255},
      smooth=Smooth.None));

  connect(TIn1.port_b, Pipe1.port_a) annotation (Line(
      points={{-60,60},{22,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(Pipe1.port_b, TOut1.port_a) annotation (Line(
      points={{42,60},{60,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TOut2.port_a, Pipe2.port_b) annotation (Line(
      points={{-60,-60},{22,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(Pipe2.port_a, TIn2.port_b) annotation (Line(
      points={{42,-60},{60,-60}},
      color={0,127,255},
      smooth=Smooth.None));

  for n in 1:nSeg loop
    connect(Tg, prescribedTemperature[n].T) annotation (Line(
      points={{0,-142},{0,-100},{-88,-100},{-88,0},{-62,0}},
      color={0,0,127},
      smooth=Smooth.None));
  end for;
  connect(Pipe1.heatPorts, R12m.port_a) annotation (Line(
      points={{32,55},{32,10}},
      color={127,0,0},
      smooth=Smooth.None));
  connect(Pipe2.heatPorts, R12m.port_b) annotation (Line(
      points={{32,-55},{32,-10}},
      color={127,0,0},
      smooth=Smooth.None));
  connect(R1.port_a, Pipe1.heatPorts) annotation (Line(
      points={{0,34},{0,40},{32,40},{32,55}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(R2.port_a, Pipe2.heatPorts) annotation (Line(
      points={{0,-34},{0,-40},{32,-40},{32,-55}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(R1.port_b, prescribedTemperature.port) annotation (Line(
      points={{-1.77636e-015,14},{0,14},{0,0},{-40,0}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(prescribedTemperature.port, R2.port_b) annotation (Line(
      points={{-40,0},{0,0},{0,-10},{1.77636e-015,-10},{1.77636e-015,-14}},
      color={191,0,0},
      smooth=Smooth.None));
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
                                 Diagram(coordinateSystem(extent={{-100,-140},{100,
            140}},      preserveAspectRatio=false),
                    graphics),
              Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -120},{100,120}}), graphics));
end DistrictHeatingPipeTR;
