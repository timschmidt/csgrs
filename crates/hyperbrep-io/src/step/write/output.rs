use super::{WriteError, WriteErrorKind, WriteLimits, WriteResource, error::Site, number};
use hyperbrep::Real;

pub(super) struct Output {
    pub text: String,
    pub site: Site,
    pub limits: WriteLimits,
    entities: usize,
    edge_uses: usize,
}
impl Output {
    pub fn new(limits: WriteLimits) -> Self {
        Self {
            text: String::new(),
            site: Site::default(),
            limits,
            entities: 0,
            edge_uses: 0,
        }
    }
    pub fn put(&mut self, text: &str) -> Result<(), WriteError> {
        let size = self.text.len().checked_add(text.len()).ok_or_else(|| {
            self.site.error(WriteErrorKind::Limit {
                resource: WriteResource::OutputBytes,
                limit: self.limits.output_bytes,
                observed: usize::MAX,
            })
        })?;
        self.site
            .check(WriteResource::OutputBytes, size, self.limits.output_bytes)?;
        self.text
            .try_reserve(text.len())
            .map_err(|_| self.site.error(WriteErrorKind::Allocation))?;
        self.text.push_str(text);
        Ok(())
    }
    pub fn entity(
        &mut self,
        name: &str,
        attributes: impl FnOnce(&mut Self) -> Result<(), WriteError>,
    ) -> Result<u64, WriteError> {
        let count = self.entities.checked_add(1).ok_or_else(|| {
            self.site.error(WriteErrorKind::Limit {
                resource: WriteResource::Entities,
                limit: self.limits.entities,
                observed: usize::MAX,
            })
        })?;
        self.site
            .check(WriteResource::Entities, count, self.limits.entities)?;
        let id = u64::try_from(count).map_err(|_| {
            self.site
                .error(WriteErrorKind::Invalid("STEP entity ID exceeds u64"))
        })?;
        self.entities = count;
        self.reference(id)?;
        self.put("=")?;
        self.put(name)?;
        self.put("(")?;
        attributes(self)?;
        self.put(");\n")?;
        Ok(id)
    }
    pub fn reference(&mut self, id: u64) -> Result<(), WriteError> {
        self.put("#")?;
        self.put(&id.to_string())
    }
    /// One complex entity instance; all components share one charged ID.
    pub fn complex_entity(
        &mut self,
        components: impl FnOnce(&mut Self) -> Result<(), WriteError>,
    ) -> Result<u64, WriteError> {
        self.entity("", components)
    }
    pub fn references(&mut self, ids: &[u64]) -> Result<(), WriteError> {
        self.put("(")?;
        for (index, id) in ids.iter().enumerate() {
            if index > 0 {
                self.put(",")?;
            }
            self.reference(*id)?;
        }
        self.put(")")
    }
    pub fn number(&mut self, value: &Real) -> Result<(), WriteError> {
        let text = number::decimal(value, &self.limits, self.site)?;
        self.put(&text)
    }
    pub fn string(&mut self, text: &str) -> Result<(), WriteError> {
        self.site.check(
            WriteResource::StringBytes,
            text.len().saturating_add(2),
            32_769,
        )?;
        if text
            .bytes()
            .all(|byte| (b' '..=b'~').contains(&byte) && byte != b'\'' && byte != b'\\')
        {
            self.put("'")?;
            self.put(text)?;
            return self.put("'");
        }
        let start = self.text.len();
        self.put("'")?;
        for character in text.chars() {
            let encoded = match character {
                '\'' => String::from("''"),
                '\\' => String::from("\\\\"),
                ' '..='~' => character.to_string(),
                '\0'..='\u{1f}' | '\u{7f}' => format!("\\X\\{:02X}", character as u32),
                '\u{80}'..='\u{ffff}' => format!("\\X2\\{:04X}\\X0\\", character as u32),
                _ => format!("\\X4\\{:08X}\\X0\\", character as u32),
            };
            let observed = (self.text.len() - start)
                .saturating_add(encoded.len())
                .saturating_add(1);
            self.site
                .check(WriteResource::StringBytes, observed, 32_769)?;
            self.put(&encoded)?;
        }
        self.put("'")
    }
    pub fn segment(&mut self) -> Result<(), WriteError> {
        let count = self.edge_uses.checked_add(1).ok_or_else(|| {
            self.site.error(WriteErrorKind::Limit {
                resource: WriteResource::EdgeUses,
                limit: self.limits.edge_uses,
                observed: usize::MAX,
            })
        })?;
        self.site
            .check(WriteResource::EdgeUses, count, self.limits.edge_uses)?;
        self.edge_uses = count;
        Ok(())
    }
}
