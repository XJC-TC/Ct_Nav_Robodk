import pytest

from ct_nav.units import UnitError, parse_deg, parse_mm


@pytest.mark.parametrize(
    "raw, expected",
    [
        ("8240 mm", 8240.0),
        ("8240mm", 8240.0),
        ("  -0.5 mm ", -0.5),
        ("1.5 m", 1500.0),
        ("2 cm", 20.0),
        ("0", 0.0),  # bare numbers appear as `x: 0` in cluster_config.yaml
        (0, 0.0),
        (18, 18.0),
        (473.5, 473.5),
    ],
)
def test_parse_mm(raw, expected):
    assert parse_mm(raw) == pytest.approx(expected)


@pytest.mark.parametrize(
    "raw, expected",
    [
        ("-90.0 deg", -90.0),
        ("113.379 deg", 113.379),
        ("0.0deg", 0.0),
        (-90, -90.0),
        ("1 rad", 57.29577951308232),
    ],
)
def test_parse_deg(raw, expected):
    assert parse_deg(raw) == pytest.approx(expected)


def test_scientific_notation():
    assert parse_mm("1e3 mm") == pytest.approx(1000.0)


@pytest.mark.parametrize("raw", ["", "mm", "abc", "12 34 mm", None, [1], True])
def test_unparseable_values_raise(raw):
    with pytest.raises(UnitError):
        parse_mm(raw)


def test_wrong_dimension_raises():
    with pytest.raises(UnitError):
        parse_mm("90 deg")
    with pytest.raises(UnitError):
        parse_deg("90 mm")


def test_error_mentions_the_field():
    with pytest.raises(UnitError, match="highway.iom_0.x"):
        parse_mm("nonsense", "highway.iom_0.x")
