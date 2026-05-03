"""Test that C2EngineNode builds its weapon list from FLEET, not from a literal block."""


def test_c2_engine_weapons_match_fleet():
    """Importing the module must not change behavior, but the literal _DEFAULT_HOMES
    block must be gone — confirm by grepping the source."""
    import inspect
    from acsdg_c2 import c2_engine_node
    src = inspect.getsource(c2_engine_node)
    assert "_DEFAULT_HOMES" not in src, (
        "c2_engine_node still references _DEFAULT_HOMES — should be sourced from FLEET")
    # And FLEET must be imported
    assert "from acsdg_c2.fleet import" in src
