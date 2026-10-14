"""Opt-in console events, chosen per node in cortex_bringup/config/logging.yaml.

The launch file turns a node's ``events: [...]`` list into its ``log_events``
string parameter. A topic named there prints its events at INFO, one line each:

    PRECHECK_RESULT {"plan_id": "p-1-0001", "target": "cucumber", "found": true}

so a console log greps and parses as-is (``grep PRECHECK_RESULT | cut -d' ' -f2- | jq``).
Topics not named stay silent; ``all`` turns every topic of the node on.
"""
import json


def parse_topics(value) -> frozenset:
    """'plan, precheck' or ['plan', 'precheck'] -> {'plan', 'precheck'}."""
    if isinstance(value, str):
        value = value.replace(',', ' ').split()
    return frozenset(str(v).strip() for v in value or () if str(v).strip())


class EventLog:
    def __init__(self, node, known, topics=None):
        """known: the topics this node emits; an unknown name in logging.yaml is warned once."""
        if topics is None:
            if not node.has_parameter('log_events'):
                node.declare_parameter('log_events', '')
            topics = node.get_parameter('log_events').value
        self.topics = parse_topics(topics)
        self._node = node
        unknown = sorted(self.topics - set(known) - {'all'})
        if unknown:
            node.get_logger().warning(f'log_events: unknown topic(s) {unknown}; known: {sorted(known)}')

    def on(self, topic: str) -> bool:
        return topic in self.topics or 'all' in self.topics

    def __call__(self, topic: str, event: str, **fields) -> None:
        if self.on(topic):
            self._node.get_logger().info(event + ' ' + json.dumps(fields, ensure_ascii=False, default=str))
