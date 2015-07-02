within DistrictHeating.Pipes.Examples;
model MixingVolume
  import DistrictHeating;
  extends Modelica.Icons.Example;
  import IDEAS;

  Buildings.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=273.15 + 20)
    annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
  IDEAS.Fluid.Sources.FixedBoundary bou6(
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    nPorts=1) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-96,-24})));
  IDEAS.Fluid.Sources.FixedBoundary bou7(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={88,-24})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TVolIn(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-22,-24})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TVolOut(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={58,-24})));
  Annex60.Fluid.MixingVolumes.MixingVolume vol(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=1,
    nPorts=2,
    V=0.098)
    annotation (Placement(transformation(extent={{46,-24},{26,-4}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalResistor thermalResistor1(R=
        2.06/50) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={46,50})));
  Annex60.Fluid.Movers.FlowControlled_m_flow fan2(redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{-58,-34},{-38,-14}})));
  Annex60.Fluid.HeatExchangers.HeaterCooler_T    idealHeater2(
    dp_nominal=0,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{-84,-34},{-64,-14}})));
  Annex60.Fluid.MixingVolumes.MixingVolume vol1(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=1,
    nPorts=2,
    V=0.098)
    annotation (Placement(transformation(extent={{22,-24},{2,-4}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalResistor thermalResistor2(R=
        2.06/50) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={22,52})));
  Modelica.Blocks.Sources.Pulse pulse(
    width=5,
    period=86400,
    offset=273.15 + 50,
    amplitude=20)
    annotation (Placement(transformation(extent={{-68,-4},{-76,4}})));
  Modelica.Blocks.Sources.Pulse pulse1(
    period=86400,
    amplitude=0,
    width=0,
    offset=0.1)
    annotation (Placement(transformation(extent={{-62,-4},{-54,4}})));
equation
  connect(TVolOut.port_b, bou7.ports[1]) annotation (Line(
      points={{68,-24},{78,-24}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(vol.ports[1], TVolOut.port_a) annotation (Line(
      points={{38,-24},{48,-24}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(vol.heatPort, thermalResistor1.port_b) annotation (Line(
      points={{46,-14},{46,40}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(TVolIn.port_a, fan2.port_b) annotation (Line(
      points={{-32,-24},{-38,-24}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(bou6.ports[1], idealHeater2.port_a) annotation (Line(
      points={{-92,-24},{-84,-24}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater2.port_b, fan2.port_a) annotation (Line(
      points={{-64,-24},{-58,-24}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TVolIn.port_b, vol1.ports[1]) annotation (Line(
      points={{-12,-24},{14,-24}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(vol1.ports[2], vol.ports[2]) annotation (Line(
      points={{10,-24},{34,-24}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(thermalResistor2.port_b, vol1.heatPort) annotation (Line(
      points={{22,42},{22,-14}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(fixedTemperature.port, thermalResistor1.port_a) annotation (Line(
      points={{-40,70},{46,70},{46,60}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(thermalResistor2.port_a, thermalResistor1.port_a) annotation (Line(
      points={{22,62},{22,70},{46,70},{46,60}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(pulse.y, idealHeater2.TSet) annotation (Line(
      points={{-76.4,0},{-92,0},{-92,-18},{-86,-18}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pulse1.y, fan2.m_flow_in) annotation (Line(
      points={{-53.6,0},{-48.2,0},{-48.2,-12}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(coordinateSystem(extent={{-100,
            -100},{100,100}})),
    experiment(StopTime=25000),
    __Dymola_experimentSetupOutput,
    __Dymola_Commands(executeCall(ensureSimulated=true) = RunScript(
        "C:/Users/u0098668/Documents/Modelica/DistrictHeating/DistrictHeating/simulate and plot.mos")
        "Simulate and plot"));
end MixingVolume;
