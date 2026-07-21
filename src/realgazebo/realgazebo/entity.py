from dataclasses import dataclass

from .type_codes import PROP_CODE_MIN, code_for_type


@dataclass(frozen=True)
class Entity:
    """Identity of one spawnable thing: a PX4 vehicle or a static prop.

    A pure value object shared by every spawn/despawn/move path. The kind
    split (is_prop) is DERIVED from type_code instead of stored, so it can
    never disagree with the code map, and name is the single definition of
    the '{type}_{id}' convention used by gz model names, the wire, and
    container discovery.
    """

    type: str       # template/type name, e.g. 'x500', 'rock'
    id: int         # instance id, globally unique across all types
    type_code: int  # wire code (u8); >= PROP_CODE_MIN marks a prop

    @property
    def name(self) -> str:
        return f'{self.type}_{self.id}'

    @property
    def is_prop(self) -> bool:
        return self.type_code >= PROP_CODE_MIN

    @classmethod
    def create(cls, entity_type, entity_id, code_map):
        """Validated constructor: resolve type_code from the scanned map.

        Raises ValueError for a type absent from the map — fail-closed,
        unlike the old _is_prop_type() which silently treated unknown
        types as PX4 vehicles and crashed later inside the backend.
        """
        return cls(entity_type, int(entity_id),
                   code_for_type(entity_type, code_map))
