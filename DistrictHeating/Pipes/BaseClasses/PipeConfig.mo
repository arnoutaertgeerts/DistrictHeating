within DistrictHeating.Pipes.BaseClasses;
package PipeConfig
  "Contains different configuration records of district heating pipes"

  record PipeDimensions "Contains pipe measurements from IsoPlus catalog"

    parameter Modelica.SIunits.Length Di = 0.1 "Equivalent inner diameter";
    parameter Modelica.SIunits.Length Do = Di "Equivalent outer diameter";

    parameter Modelica.SIunits.Length h = Di
      "Horizontal distance between pipe walls";
    parameter Modelica.SIunits.Length Dc = 2.5*Di
      "Diameter of circumscribing pipe";

    final parameter Modelica.SIunits.Length E = h + Di
      "Horizontal distance between pipe centers";

  end PipeDimensions;

  record IsoPlusDR20S "Standard DN 20 IsoPlus double pipe"
    extends PipeDimensions(
      Di=20e-3,
      Do=26.9e-3,
      h=20e-3,
      Dc=125e-3);
  end IsoPlusDR20S;

  record IsoPlusDR25S "Standard DN 25 IsoPlus double pipe"
    extends PipeDimensions(
      h=20e-3,
      Di=25e-3,
      Do=33.7e-3,
      Dc=140e-3);
  end IsoPlusDR25S;

  record IsoPlusDR32S "Standard DN 32 IsoPlus double pipe"
    extends PipeDimensions(
      h=20e-3,
      Di=32e-3,
      Do=42.4e-3,
      Dc=160e-3);
  end IsoPlusDR32S;

  record IsoPlusDR40S "Standard DN 40 IsoPlus double pipe"
    extends PipeDimensions(
      h=20e-3,
      Di=40e-3,
      Do=48.3e-3,
      Dc=160e-3);
  end IsoPlusDR40S;

  record IsoPlusDR50S "Standard DN 50 IsoPlus double pipe"
    extends PipeDimensions(
      h=20e-3,
      Di=50e-3,
      Do=60.3e-3,
      Dc=200e-3);
  end IsoPlusDR50S;

  record IsoPlusDR65S "Standard DN 65 IsoPlus double pipe"
    extends PipeDimensions(
      h=20e-3,
      Di=65e-3,
      Do=76.1e-3,
      Dc=225e-3);
  end IsoPlusDR65S;

  record IsoPlusDR80S "Standard DN 80 IsoPlus double pipe"
    extends PipeDimensions(
      Di=80e-3,
      Do=88.9e-3,
      h=25e-3,
      Dc=250e-3);
  end IsoPlusDR80S;

  record IsoPlusDR100S "Standard DN 100 IsoPlus double pipe"
    extends PipeDimensions(
      h=25e-3,
      Di=100e-3,
      Do=114.3e-3,
      Dc=315e-3);
  end IsoPlusDR100S;

  record IsoPlusDR125S "Standard DN 125 IsoPlus double pipe"
    extends PipeDimensions(
      Di=125e-3,
      Do=139.7e-3,
      h=30e-3,
      Dc=400e-3);
  end IsoPlusDR125S;

  record IsoPlusDR150S "Standard DN 150 IsoPlus double pipe"
    extends PipeDimensions(
      Di=150e-3,
      Do=168.3e-3,
      h=40e-3,
      Dc=450e-3);
  end IsoPlusDR150S;

  record IsoPlusDR200S "Standard DN 200 IsoPlus double pipe"
    extends PipeDimensions(
      Di=200e-3,
      Do=219.1e-3,
      h=45e-3,
      Dc=560e-3);
  end IsoPlusDR200S;
end PipeConfig;