within DistrictHeating.Pipes.BaseClasses;
partial model DHDeltaCircuit
  "District heating pipe model based on the resistor delta circuit"

  //Extensions
  extends PartialDistrictHeatingPipe;

  //Parameters
  parameter Integer nSeg=5;

  final parameter Types.ThermalResistanceLength Rs = 1/(2*Modelica.Constants.pi*lambdaI*hs);
  final parameter Types.ThermalResistanceLength Ra = 1/(2*Modelica.Constants.pi*lambdaI*ha);

  final parameter Types.ThermalResistanceLength R12 = (2*Ra*Rs)/(Rs-Ra);
  final parameter Types.ThermalResistanceLength Rbou = Rs;

  //Components
  Buildings.Fluid.FixedResistances.Pipe      Pipe1(
    redeclare package Medium = Medium,
    m_flow_nominal=m1_flow_nominal,
    dp_nominal=dp_nominal*L,
    massDynamics=massDynamics,
    energyDynamics=energyDynamics,
    length=L,
    diameter=Di,
    nSeg=nSeg,
    useMultipleHeatPorts=true,
    lambdaIns=lambdaI,
    thicknessIns=10e-10)
    annotation (Placement(transformation(extent={{10,50},{-10,70}})));
  Buildings.Fluid.FixedResistances.Pipe      Pipe2(
    redeclare package Medium = Medium,
    massDynamics=massDynamics,
    energyDynamics=energyDynamics,
    m_flow_nominal=m2_flow_nominal,
    dp_nominal=dp_nominal*L,
    length=L,
    diameter=Di,
    nSeg=nSeg,
    useMultipleHeatPorts=true,
    lambdaIns=lambdaI,
    thicknessIns=10e-10)
    annotation (Placement(transformation(extent={{10,-50},{-10,-70}})));

  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature[nSeg]
    prescribedTemperature annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,0})));
  Modelica.Thermal.HeatTransfer.Components.ThermalResistor R12m[nSeg](R=R12*
        nSeg/L)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,0})));
  Modelica.Thermal.HeatTransfer.Components.ThermalResistor R1[nSeg](R=Rbou*nSeg/
        L)                                                          annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-20,24})));
  Modelica.Thermal.HeatTransfer.Components.ThermalResistor R2[nSeg](R=Rbou*nSeg/
        L)                                                          annotation (
     Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={-20,-24})));
equation

  for i in 1:nSeg loop
    connect(Tg, prescribedTemperature[i].T) annotation (Line(
      points={{0,-142},{0,-100},{-88,-100},{-88,0},{-62,0}},
      color={0,0,127},
      smooth=Smooth.None));
  end for;

  connect(Pipe2.port_b, port_b2) annotation (Line(
      points={{-10,-60},{-100,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(Pipe2.port_a, port_a2) annotation (Line(
      points={{10,-60},{100,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(Pipe1.port_b, port_a1) annotation (Line(
      points={{-10,60},{-100,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(Pipe1.port_a, port_b1) annotation (Line(
      points={{10,60},{100,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(R12m.port_a, Pipe1.heatPorts) annotation (Line(
      points={{0,10},{0,55}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(R12m.port_b, Pipe2.heatPorts) annotation (Line(
      points={{0,-10},{0,-55}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(R1.port_a, Pipe1.heatPorts) annotation (Line(
      points={{-20,34},{-20,40},{0,40},{0,55}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(R2.port_a, Pipe2.heatPorts) annotation (Line(
      points={{-20,-34},{-20,-40},{0,-40},{0,-55}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(R2.port_b, R1.port_b) annotation (Line(
      points={{-20,-14},{-20,14}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(prescribedTemperature.port, R1.port_b) annotation (Line(
      points={{-40,0},{-20,0},{-20,14}},
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
end DHDeltaCircuit;
