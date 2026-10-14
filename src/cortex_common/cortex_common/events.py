"""Opt-in events, chosen per node in cortex_bringup/config/logging.yaml.

The launch file turns logging.yaml into two string parameters per node:

    log_events     topics printed on the console at INFO, one line each:
                   PRECHECK_RESULT {"plan_id": "p-1-0001", "target": "cucumber", "found": true}
    record_events  topics published on /cortex/events (std_msgs/String, one JSON object
                   per event) for the session recorder, whatever the console shows

so a console log greps and parses as-is (``grep PRECHECK_RESULT | cut -d' ' -f2- | jq``).
Topics not named stay silent; ``all`` turns every topic of the node on.
"""
import json

EVENTS_TOPIC = '/cortex/events'


def parse_topics(value) -> frozenset:
    """'plan, precheck' or ['plan', 'precheck'] -> {'plan', 'precheck'}."""
    if isinstance(value, str):
        value = value.replace(',', ' ').split()
    return frozenset(str(v).strip() for v in value or () if str(v).strip())


def _param(node, name):
    if not node.has_parameter(name):
        node.declare_parameter(name, '')
    return node.get_parameter(name).value


class EventLog:
    def __init__(self, node, known, topics=None, record=None):
        """known: the topics this node emits; an unknown name in logging.yaml is warned once.
        topics/record override the parameters (tests); record defaults to off then."""
        if topics is None:
            topics = _param(node, 'log_events')
            record = _param(node, 'record_events') if record is None else record
        self.topics = parse_topics(topics)
        self.record = parse_topics(record or '')
        self._node = node
        self._pub = None
        if self.record:
            from std_msgs.msg import String
            self._msg = String
            self._pub = node.create_publisher(String, EVENTS_TOPIC, 100)
        unknown = sorted((self.topics | self.record) - set(known) - {'all'})
        if unknown:
            node.get_logger().warning(f'log_events: unknown topic(s) {unknown}; known: {sorted(known)}')

    @staticmethod
    def _has(topics, topic):
        return topic in topics or 'all' in topics

    def on(self, topic: str) -> bool:
        """Is anyone (console or recorder) listening? Gate costly field building on this."""
        return self._has(self.topics, topic) or self._has(self.record, topic)

    def __call__(self, topic: str, event: str, **fields) -> None:
        if self._has(self.topics, topic):
            self._node.get_logger().info(event + ' ' + json.dumps(fields, ensure_ascii=False, default=str))
        if self._pub is not None and self._has(self.record, topic):
            body = {'stamp_ns': self._node.get_clock().now().nanoseconds,
                    'node': self._node.get_name(), 'topic': topic, 'event': event, 'fields': fields}
            self._pub.publish(self._msg(data=json.dumps(body, ensure_ascii=False, default=str)))
