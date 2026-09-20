import pytest
from metro_perception_tools.metrics import summarize


def test_unknown_does_not_become_a_true_negative():
    summary = summarize([
        {'state': 'UNKNOWN', 'processing_ms': 1, 'mode': 'scaffold'},
        {'state': 'NO_OBSTACLE_DETECTED', 'processing_ms': 2},
    ])
    assert summary['unknown_fraction'] == 0.5
    assert summary['quality_metrics'] is None
    assert summary['scaffold']


def test_invalid_or_empty_results_fail_instead_of_reporting_success():
    with pytest.raises(ValueError):
        summarize([])
    with pytest.raises(ValueError):
        summarize([{'state': 'UNKNOWN', 'processing_ms': float('nan')}])


def test_a02_is_not_a_completed_detector():
    result = summarize([{'state': 'UNKNOWN', 'processing_ms': 1, 'mode': 'a02'}])
    assert result['scaffold']
    assert result['quality_metrics'] is None
